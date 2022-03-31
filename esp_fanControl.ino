/*  esp_shadecontrol
Code used to control a Fanimation Slinger V2 RF Fan via an ESP8266 and a TI CC1101 chip over MQTT

ESP8266 CC1101 Wiring: https://github.com/LSatan/SmartRC-CC1101-Driver-Lib/blob/master/img/Esp8266_CC1101.png

code conversion started
changing fan to shade
commenting out light related code

things we want to do with the shade


shade 
all the way up
all the way down
lower to preset 

Can probably remove all the code related to fan modes
need to look at reporting state for a cover 
convert over to the SOMFY commands 


*/

#include "config.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h> //for CC1101
#include <RCSwitch.h> //for CC1101
#include <ESP8266WiFi.h> //for ESP8266
#include <PubSubClient.h> //for MQTT
#include <ArduinoJson.h> //Used for MQTT Autodiscovery Payload Creation

// Set receive and transmit pin numbers (GDO0 and GDO2)
#ifdef ESP32 // for esp32! Receiver on GPIO pin 4. Transmit on GPIO pin 2.
  #define RX_PIN 4 
  #define TX_PIN 2
#elif ESP8266  // for esp8266! Receiver on pin 4 = D2. Transmit on pin 5 = D1.
  #define RX_PIN 4
  #define TX_PIN 5
#else // for Arduino! Receiver on interrupt 0 => that is pin #2. Transmit on pin 6.
  #define RX_PIN 0
  #define TX_PIN 6
#endif 

#define DELETE(ptr) { if (ptr != nullptr) {delete ptr; ptr = nullptr;} }

//MQTT TOPICS
String INFO_TOPIC = (String)MQTT_BASETOPIC + "/sensor/shadecontrol_" + String(ESP.getChipId()) + "/info/attributes";
String INFO_CONFIG_TOPIC = (String)MQTT_BASETOPIC + "/sensor/shadecontrol_" + String(ESP.getChipId()) + "/info/config";
String AVAILABILITY_TOPIC = (String)MQTT_BASETOPIC + "/sensor/shadecontrol_" + String(ESP.getChipId()) + "/status";

/* hacking out light stuff
String LIGHT_STATE_TOPIC = (String)MQTT_BASETOPIC + "/light/shadecontrol_" + String(ESP.getChipId()) + "/state";
String LIGHT_COMMAND_TOPIC = (String)MQTT_BASETOPIC + "/light/shadecontrol_" + String(ESP.getChipId()) + "/set";
String LIGHT_CONFIG_TOPIC = (String)MQTT_BASETOPIC + "/light/shadecontrol_" + String(ESP.getChipId())+"/config";
*/

String SHADE_STATE_TOPIC = (String)MQTT_BASETOPIC + "/shade/shadecontrol_" + String(ESP.getChipId()) + "/state";
String SHADE_COMMAND_TOPIC = (String)MQTT_BASETOPIC + "/shade/shadecontrol_" + String(ESP.getChipId()) + "/set";
String SHADE_POSITION_STATE_TOPIC = (String)MQTT_BASETOPIC + "/shade/shadecontrol_" + String(ESP.getChipId()) +"/position/percentage_state";
String SHADE_POSITION_COMMAND_TOPIC = (String)MQTT_BASETOPIC + "/shade/shadecontrol_" + String(ESP.getChipId()) + "/position/percentage";
String SHADE_MODE_STATE_TOPIC = (String)MQTT_BASETOPIC + "/shade/shadecontrol_" + String(ESP.getChipId()) + "/preset/preset_mode_state";
String SHADE_MODE_COMMAND_TOPIC = (String)MQTT_BASETOPIC + "/shade/shadecontrol_" + String(ESP.getChipId()) + "/preset_mode";
String SHADE_CONFIG_TOPIC = (String)MQTT_BASETOPIC + "/shade/shadecontrol_" + String(ESP.getChipId())+"/config";

//Variable to track CC1101 State
bool CC1101_RX_ON = true;

//Variables used for tracking Fan State
bool SUMMER_MODE = true;
// bool CURRENT_LIGHT_STATE = false;
bool CURRENT_SHADE_STATE = false;
int CURRENT_SHADE_POSITION = 0;

bool CC1101_CONNECTED = false;
RCSwitch shadecontrolClient = RCSwitch();

WiFiClient *client = nullptr;
PubSubClient *mqtt_client = nullptr;
static String deviceStr ="";

#pragma region System_Or_Helper_Functions
//Function for keeping track of system uptime.
String getSystemUptime()
{
  long millisecs = millis();
  int systemUpTimeMn = int((millisecs / (1000 * 60)) % 60);
  int systemUpTimeHr = int((millisecs / (1000 * 60 * 60)) % 24);
  int systemUpTimeDy = int((millisecs / (1000 * 60 * 60 * 24)) % 365);
  return String(systemUpTimeDy)+"d:"+String(systemUpTimeHr)+"h:"+String(systemUpTimeMn)+"m";
}

//Logging helper function
void SHADECONTORL_LOGGER(String logMsg, int requiredLVL, bool addNewLine)
{
  if(requiredLVL <= SHADE_DEBUG_LVL)
  {
    if(addNewLine)
      Serial.printf("%s\n", logMsg.c_str());
    else
      Serial.printf("%s", logMsg.c_str());
  }
}

//Toggle LED State
void toggleLED()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

//Check and validate MQTT is connected and 
void heartBeatPrint()
{
  if(mqtt_client != nullptr) //We have to check and see if we have a mqtt client created
  {
    if(!mqtt_client->connected())
    {
      SHADECONTORL_LOGGER("MQTT Disconnected", 0, true);
      connectMQTT();
    }
  }
  SHADECONTORL_LOGGER(String("free heap memory: ") + String(ESP.getFreeHeap()), 4, true);
}

//Function to check status of Wifi and MQTT
void check_status()
{
  static ulong checkstatus_timeout  = 0;
  static ulong LEDstatus_timeout    = 0;
  static ulong checkwifi_timeout    = 0;
  static ulong shadecontrolheartbeat_timeout = 0;
  ulong current_millis = millis();

  // Check WiFi every WIFICHECK_INTERVAL (1) seconds.
  if ((current_millis > checkwifi_timeout) || (checkwifi_timeout == 0))
  {
    check_WiFi();
    mqtt_client->loop();
    checkwifi_timeout = current_millis + WIFICHECK_INTERVAL;
  }

  if ((current_millis > LEDstatus_timeout) || (LEDstatus_timeout == 0))
  {
    // Toggle LED at LED_INTERVAL = 2s
    toggleLED();
    LEDstatus_timeout = current_millis + LED_INTERVAL;
  }
  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((current_millis > checkstatus_timeout) || (checkstatus_timeout == 0))
  { 
    heartBeatPrint();
    checkstatus_timeout = current_millis + HEARTBEAT_INTERVAL;
  }

  // Print shadecontrol System Info every shadecontrolHEARTBEAT_INTERVAL (5) minutes.
  if ((current_millis > shadecontrolheartbeat_timeout) || (shadecontrolheartbeat_timeout == 0))
  {
    publishSystemInfo();
    shadecontrolheartbeat_timeout = current_millis + shadecontrolHEARTBEAT_INTERVAL;
  }
}
#pragma endregion
#pragma region Wifi_Related_Functions
void check_WiFi()
{
  if ( (WiFi.status() != WL_CONNECTED) )
  {
    SHADECONTORL_LOGGER("WiFi Connection Lost!",0,true);
    disconnectMQTT();
    connectWiFi();
  }
}

void connectWiFi() 
{
  delay(10);
  SHADECONTORL_LOGGER("Connecting to "+String(WIFI_SSID),0, false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    SHADECONTORL_LOGGER(".",0, false);
  }
  SHADECONTORL_LOGGER("",0,true);
  randomSeed(micros());
  SHADECONTORL_LOGGER("WiFi connected",0,true);
  SHADECONTORL_LOGGER("IP: "+ WiFi.localIP().toString(),0,true);
  
}
#pragma endregion
#pragma region RF_Related_Functions
//Function used to send the RF Command requested
void sendRFCommand(int code)
{
  shadecontrolClient.disableReceive(); //Turn Off Listening
  CC1101_RX_ON = false;
  SHADECONTORL_LOGGER("[RX] RX LISTENING: OFF", 4, true);
  shadecontrolClient.enableTransmit(TX_PIN); //Enable RF TX                                                  
  ELECHOUSE_cc1101.SetTx();
  SHADECONTORL_LOGGER("[TX] Transmitting RF Code " + String(code), 2, true);
  shadecontrolClient.setRepeatTransmit(RF_REPEATS);
  shadecontrolClient.setProtocol(RF_PROTOCOL);
  shadecontrolClient.setPulseLength(382);
  shadecontrolClient.send(code, 24);
  SHADECONTORL_LOGGER("[TX] Transmission Complete!", 2, true);
  ELECHOUSE_cc1101.SetRx();
  shadecontrolClient.disableTransmit();
  shadecontrolClient.enableReceive(RX_PIN);
}

//Call this function to get the codes for the buttons we want to use
void startLearningMode()
{
  if(!CC1101_RX_ON)
  {
    shadecontrolClient.enableReceive(RX_PIN);
    ELECHOUSE_cc1101.SetRx();
    CC1101_RX_ON = true;
    SHADECONTORL_LOGGER("[RX] RX LISTENING: ON", 4, true);
    shadecontrolClient.resetAvailable();
  }

  if(shadecontrolClient.available())
  {
    SHADECONTORL_LOGGER("[LEARN] !!! BUTTON PRESS DETECTED !!!", 1, true);
    SHADECONTORL_LOGGER("[LEARN] Code: "+String(shadecontrolClient.getReceivedValue()), 1, true);
    SHADECONTORL_LOGGER("[LEARN] Bit: "+String(shadecontrolClient.getReceivedBitlength()), 1, true);
    SHADECONTORL_LOGGER("[LEARN] Protocol: "+String(shadecontrolClient.getReceivedProtocol()), 1, true);
    SHADECONTORL_LOGGER("[LEARN] Delay: "+String(shadecontrolClient.getReceivedDelay()), 1, true);
    shadecontrolClient.resetAvailable();
  }
}

//This function is used inside the main loop to listen for codes from the remote while we are awaiting messages from MQTT
void listenForCodes()
{
  if(!CC1101_RX_ON)
  {
    shadecontrolClient.enableReceive(RX_PIN);
    ELECHOUSE_cc1101.SetRx();
    CC1101_RX_ON = true;
    SHADECONTORL_LOGGER("[RX] RX LISTENING: ON", 4, true);
    shadecontrolClient.resetAvailable();
  }

  if(shadecontrolClient.available())
  {
    String rxCode = String(shadecontrolClient.getReceivedValue());
    SHADECONTORL_LOGGER("[RX] Code: " + rxCode, 3, true);
    if(rxCode == String(LIGHT_ON))
      mqtt_client->publish(LIGHT_STATE_TOPIC.c_str(),"on",true);
    else if(rxCode == String(LIGHT_OFF))
      mqtt_client->publish(LIGHT_STATE_TOPIC.c_str(),"off",true);
    else if(rxCode == String(SHADE_OFF))
      mqtt_client->publish(SHADE_STATE_TOPIC.c_str(),"off",true);
    else if(rxCode == String(SUMMER_SHADE_ON) )
    {
      mqtt_client->publish(SHADE_STATE_TOPIC.c_str(),"on",true);
      mqtt_client->publish(SHADE_MODE_STATE_TOPIC.c_str(),"Summer",true);
    }
    else if(rxCode == String(WINTER_SHADE_ON))
    {
      mqtt_client->publish(SHADE_STATE_TOPIC.c_str(),"on",true);
      mqtt_client->publish(SHADE_MODE_STATE_TOPIC.c_str(),"Winter",true);
    }
    else if( rxCode.toInt() >= LIGHT_MIN  && rxCode.toInt() <= LIGHT_MAX )
      mqtt_client->publish(LIGHT_STATE_TOPIC.c_str(),"on",true);
    else if( rxCode.toInt() >= SUMMER_SHADE_MIN  && rxCode.toInt() <= SUMMER_SHADE_MAX )
    {
      mqtt_client->publish(SHADE_STATE_TOPIC.c_str(),"on",true);
      int speed = rxCode.toInt() - SUMMER_SHADE_MIN;
      mqtt_client->publish(SHADE_POSITION_STATE_TOPIC.c_str(),String(speed).c_str(),true);
    }
    else if( rxCode.toInt() >= WINTER_SHADE_MIN  && rxCode.toInt() <= WINTER_SHADE_MAX )
    {
      mqtt_client->publish(SHADE_STATE_TOPIC.c_str(),"on",true);
      int speed = rxCode.toInt() - WINTER_SHADE_MIN;
      mqtt_client->publish(SHADE_POSITION_STATE_TOPIC.c_str(),String(speed).c_str(),true);
    }
    else if(rxCode == String(SUMMER_SHADE_MODE))
    {
      mqtt_client->publish(SHADE_MODE_STATE_TOPIC.c_str(),"Summer",true);
    }
    else if(rxCode == String(WINTER_SHADE_MODE))
    {
      mqtt_client->publish(SHADE_MODE_STATE_TOPIC.c_str(),"Winter",true);
    }
    shadecontrolClient.resetAvailable();
  }
}
#pragma endregion
#pragma region MQTT_Related_Functions
void callback(char* topic, byte* payload, unsigned int length) 
{
  String payloadStr = "";
  for (int i=0;i<length;i++)
  {
    payloadStr += (char)payload[i];
  }
  SHADECONTORL_LOGGER("Message arrived ["+String(topic)+"] ("+ payloadStr +")", 3, true);
  if(String(topic) == SHADE_STATE_TOPIC )
  {
    if(payloadStr == "on")
      CURRENT_SHADE_STATE = true;
    else if(payloadStr == "off" )
      CURRENT_SHADE_STATE = false;
  }
  else if(String(topic) == LIGHT_STATE_TOPIC )
  {
    if(payloadStr == "on")
      CURRENT_LIGHT_STATE = true;
    else if(payloadStr == "off" )
      CURRENT_LIGHT_STATE = false;
  }
  else if( String(topic) == SHADE_POSITION_STATE_TOPIC )
  {
    CURRENT_SHADE_POSITION = payloadStr.toInt();
  }
  else if( String(topic) == SHADE_MODE_STATE_TOPIC )
  {
    if(payloadStr == "Summer")
    {
      SUMMER_MODE = true;
    }
    else if(payloadStr == "Winter")
    {
      SUMMER_MODE = false;
    }
  }
  /* hack light stuff
  else if(String(topic) == LIGHT_COMMAND_TOPIC )
  {
    if(payloadStr == "on")
    {
      sendRFCommand(LIGHT_ON);
      mqtt_client->publish(LIGHT_STATE_TOPIC.c_str(),"on",true);
    }
    if(payloadStr == "off")
    {
      sendRFCommand(LIGHT_OFF);
      mqtt_client->publish(LIGHT_STATE_TOPIC.c_str(),"off",true);
    }
  }
  */
 
  else if(String(topic) == SHADE_COMMAND_TOPIC )
  {
    if(payloadStr == "on")
    {
      if(SUMMER_MODE)
        sendRFCommand(SUMMER_SHADE_ON);
      else
        sendRFCommand(WINTER_SHADE_ON);
      mqtt_client->publish(SHADE_STATE_TOPIC.c_str(),"on",true);
    }
    if(payloadStr == "off")
    {
      sendRFCommand(SHADE_OFF);
      mqtt_client->publish(SHADE_STATE_TOPIC.c_str(),"off",true);
    }      
  }
  else if( String(topic) == SHADE_POSITION_COMMAND_TOPIC )
  {
    int fanSpeed = payloadStr.toInt();
    mqtt_client->publish(SHADE_STATE_TOPIC.c_str(),"on",true);
    mqtt_client->publish(SHADE_POSITION_STATE_TOPIC.c_str(),payloadStr.c_str(),true);
    if(SUMMER_MODE)
    {
      int txCode = SUMMER_SHADE_MIN + fanSpeed; 
      sendRFCommand(txCode);
    }
    else
    {
      int txCode = WINTER_SHADE_MIN + fanSpeed;
      sendRFCommand(txCode);
    }
  }
  else if( String(topic) == SHADE_MODE_COMMAND_TOPIC )
  {
    if(payloadStr == "Summer")
    {
      if(CURRENT_SHADE_STATE)
        SHADECONTORL_LOGGER("!! FAN MUST BE OFF TO CHANGE MODES !!",0,true); 
      else
      {
        sendRFCommand(SUMMER_SHADE_MODE);
        SUMMER_MODE = true;
        mqtt_client->publish(SHADE_MODE_STATE_TOPIC.c_str(),payloadStr.c_str(),true);
        
      }
    }
    else if(payloadStr == "Winter")
    {
      if(CURRENT_SHADE_STATE)
        SHADECONTORL_LOGGER("!! FAN MUST BE OFF TO CHANGE MODES !!",0,true); 
      else
      {
        sendRFCommand(WINTER_SHADE_MODE);
        SUMMER_MODE = false;
        mqtt_client->publish(SHADE_MODE_STATE_TOPIC.c_str(),payloadStr.c_str(),true);
      }
    }
  }
  else
    SHADECONTORL_LOGGER("UNKNOWN PAYLOAD: " + payloadStr,0,true);  
}

void disconnectMQTT()
{
  try
  {
    if (mqtt_client != nullptr)
    {
      if(mqtt_client->connected())
      {
        mqtt_client->disconnect();
      }
      DELETE(mqtt_client);
    }
  }
  catch(...)
  {
    SHADECONTORL_LOGGER("Error disconnecting MQTT",0,true);
  }
}

//Connect to MQTT Server
void connectMQTT() 
{
  SHADECONTORL_LOGGER("Connecting to MQTT...", 0,true);
  if (client == nullptr)
    client = new WiFiClient();
  if(mqtt_client == nullptr)
  {
    mqtt_client = new PubSubClient(*client);
    mqtt_client->setBufferSize(2048); //Needed as some JSON messages are too large for the default size
    mqtt_client->setKeepAlive(60); //Added to Stabilize MQTT Connection
    mqtt_client->setSocketTimeout(60); //Added to Stabilize MQTT Connection
    mqtt_client->setServer(MQTT_SERVER, atoi(MQTT_SERVERPORT));
    mqtt_client->setCallback(&callback);
  }
  if (!mqtt_client->connect(String(ESP.getChipId()).c_str(), MQTT_USERNAME, MQTT_PASSWORD, AVAILABILITY_TOPIC.c_str(), 1, true, "offline"))
  {
    SHADECONTORL_LOGGER("MQTT connection failed: " + String(mqtt_client->state()), 0,true);
    DELETE(mqtt_client);
    delay(1*5000); //Delay for 5 seconds after a connection failure
  }
  else
  {
    SHADECONTORL_LOGGER("MQTT connected", 1,true);
    mqtt_client->publish(AVAILABILITY_TOPIC.c_str(),"online");
    delay(500);

    /* hacking out light related stuff
    mqtt_client->subscribe(LIGHT_STATE_TOPIC.c_str());
    mqtt_client->loop();
    mqtt_client->loop();
    mqtt_client->subscribe(LIGHT_COMMAND_TOPIC.c_str());
    mqtt_client->loop();
    mqtt_client->loop();
    */
    mqtt_client->subscribe(SHADE_STATE_TOPIC.c_str());
    mqtt_client->loop();
    mqtt_client->loop();
    mqtt_client->subscribe(SHADE_COMMAND_TOPIC.c_str());
    mqtt_client->loop();
    mqtt_client->loop();
    mqtt_client->subscribe(SHADE_POSITION_STATE_TOPIC.c_str());
    mqtt_client->loop();
    mqtt_client->loop();
    mqtt_client->subscribe(SHADE_POSITION_COMMAND_TOPIC.c_str());
    mqtt_client->loop();
    mqtt_client->loop();
    mqtt_client->subscribe(SHADE_MODE_STATE_TOPIC.c_str());
    mqtt_client->loop();
    mqtt_client->loop();
    mqtt_client->subscribe(SHADE_MODE_COMMAND_TOPIC.c_str());
    mqtt_client->loop();
    mqtt_client->loop();
  }
}

//Publish shadecontrol Client info to MQTT
void publishSystemInfo()
{
  if(mqtt_client)
  {
    if(mqtt_client->connected())
    {
      SHADECONTORL_LOGGER("==== espshadecontrol Internal State ==== ",3,true);
      if(SUMMER_MODE)
        SHADECONTORL_LOGGER("[MODE]: Summer",3,true);
      else
        SHADECONTORL_LOGGER("[MODE]: Winter",3,true);
      if(CURRENT_LIGHT_STATE)
        SHADECONTORL_LOGGER("[LIGHT]: ON",3,true);
      else
        SHADECONTORL_LOGGER("[LIGHT]: OFF",3,true);
      if(CURRENT_SHADE_STATE)
        SHADECONTORL_LOGGER("[FAN]: ON",3,true);
      else
        SHADECONTORL_LOGGER("[FAN]: OFF",3,true);
      SHADECONTORL_LOGGER("[SPEED]: "+String(CURRENT_SHADE_POSITION),2,true);
      mqttAnnounce();
    }
    else
      connectMQTT();
  }
  else
    connectMQTT();
}

//Publish MQTT Configuration Topics used by MQTT Auto Discovery
void mqttAnnounce()
{
  String syspayload="";
 // String lightPayload ="";
  String shadePayload = "";
  String infoSensorPayload ="";

  DynamicJsonDocument deviceJSON(1024);
  JsonObject deviceObj = deviceJSON.createNestedObject("device");
  deviceObj["identifiers"] = String(ESP.getChipId());
  deviceObj["manufacturer"] = String(ESP.getFlashChipVendorId());
  deviceObj["model"] = "ESP8266";
  deviceObj["name"] = "shadecontrol_" + String(ESP.getChipId());
  deviceObj["sw_version"] = "1.0";
  serializeJson(deviceObj, deviceStr);
  
  DynamicJsonDocument sysinfoJSON(1024);
  sysinfoJSON["device"] = deviceObj;
  sysinfoJSON["name"] = "shadecontrol_" + String(ESP.getChipId());
  sysinfoJSON["Uptime"] = getSystemUptime();
  sysinfoJSON["Network"] = WiFi.SSID();
  sysinfoJSON["Signal Strength"] = String(WiFi.RSSI());
  sysinfoJSON["IP Address"] = WiFi.localIP().toString();
  serializeJson(sysinfoJSON,syspayload);

  DynamicJsonDocument infoSensorJSON(1024);
  infoSensorJSON["device"] = deviceObj;
  infoSensorJSON["name"] = "shadecontrol_" + String(ESP.getChipId()) + " Sensor";
  infoSensorJSON["icon"] = "mdi:chip";
  infoSensorJSON["unique_id"] = "shadecontrol_" + String(ESP.getChipId())+"_info";
  infoSensorJSON["state_topic"] = AVAILABILITY_TOPIC;
  infoSensorJSON["json_attributes_topic"] = INFO_TOPIC;
  serializeJson(infoSensorJSON,infoSensorPayload);

  DynamicJsonDocument lightJSON(1024);
  lightJSON["device"] = deviceObj;
  lightJSON["name"] = "shadecontrol_" + String(ESP.getChipId()) + " Light";
  lightJSON["unique_id"]   = "shadecontrol_" + String(ESP.getChipId())+"_light";
  lightJSON["state_topic"] = LIGHT_STATE_TOPIC;
  lightJSON["command_topic"] = LIGHT_COMMAND_TOPIC;
  lightJSON["payload_on"] = "on";
  lightJSON["payload_off"] = "off";
  lightJSON["availability_topic"] = AVAILABILITY_TOPIC;
  lightJSON["payload_available"] = "online";
  lightJSON["payload_not_available"] = "offline";
  serializeJson(lightJSON,lightPayload);
  
  DynamicJsonDocument fanJSON(1024);
  fanJSON["device"] = deviceObj;
  fanJSON["name"] = "shadecontrol_" + String(ESP.getChipId()) + " Fan";
  fanJSON["unique_id"]   = "shadecontrol_" + String(ESP.getChipId())+"_fan";
  fanJSON["state_topic"] = SHADE_STATE_TOPIC;
  fanJSON["command_topic"] = SHADE_COMMAND_TOPIC;
  fanJSON["payload_on"] = "on";
  fanJSON["payload_off"] = "off";
  fanJSON["percentage_state_topic"]  = SHADE_POSITION_STATE_TOPIC;
  fanJSON["percentage_command_topic"] = SHADE_POSITION_COMMAND_TOPIC;
  fanJSON["speed_range_min"] = 1;
  fanJSON["speed_range_max"] = 30;
  fanJSON["preset_mode_state_topic"] = SHADE_MODE_STATE_TOPIC;
  fanJSON["preset_mode_command_topic"]= SHADE_MODE_COMMAND_TOPIC;
  JsonArray SHADE_PRESET_MODES = fanJSON.createNestedArray("preset_modes");
  SHADE_PRESET_MODES.add("Summer");
  SHADE_PRESET_MODES.add("Winter");
  fanJSON["availability_topic"] = AVAILABILITY_TOPIC;
  fanJSON["payload_available"] = "online";
  fanJSON["payload_not_available"] = "offline";
  serializeJson(fanJSON,shadePayload);

  if(mqtt_client)
  {
    if(mqtt_client->connected())
    {
      mqtt_client->publish(AVAILABILITY_TOPIC.c_str(),"online");
      delay(100);
     // mqtt_client->publish(LIGHT_CONFIG_TOPIC.c_str(),lightPayload.c_str(),true);
     // delay(100);
      mqtt_client->publish(SHADE_CONFIG_TOPIC.c_str(),shadePayload.c_str(),true);
      delay(100);
      mqtt_client->publish(INFO_CONFIG_TOPIC.c_str(),infoSensorPayload.c_str(),true);
      delay(100);
      mqtt_client->publish(INFO_TOPIC.c_str(),syspayload.c_str(),true);
      delay(100);
    }
    else
    {
      connectMQTT();
    }
  }
}

#pragma endregion

//Setup Function
void setup() 
{
  //Start Serial Connection
  Serial.begin(115200);
  while (!Serial);
  delay(200);
  SHADECONTORL_LOGGER("Starting shadecontrol Client on " + String(ARDUINO_BOARD), 0, true);
  // Initialize the LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // Check the CC1101 Spi connection is working.
  if (ELECHOUSE_cc1101.getCC1101())
  {
    SHADECONTORL_LOGGER("CC1101 SPI Connection: [OK]", 0, true);
    CC1101_CONNECTED = true;
  }
  else
  {
    SHADECONTORL_LOGGER("CC1101 SPI Connection: [ERR]", 0, true);
    CC1101_CONNECTED = false;
  }
  //Initialize the CC1101 and Set the Frequency
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(FREQUENCY); 
  //Put Device in Listening Mode
  shadecontrolClient.enableReceive(RX_PIN);
  ELECHOUSE_cc1101.SetRx();
  connectWiFi();
  connectMQTT();
}

//Loop Function
void loop() 
{
  //Make sure the CC1101 Chip is connected otherwise do nothing
  if(CC1101_CONNECTED)
  {
    int counter = 0;
    if(LEARNING_MODE) 
      startLearningMode();
    else
    {
      //Check to see if someone is using the remote (We only TX when told via MQTT)
      listenForCodes();
    }
  }
  check_status();
}
