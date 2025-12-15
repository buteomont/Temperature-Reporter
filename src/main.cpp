#include <Arduino.h>
#include <U8g2lib.h>
#include "driver/adc.h"
#include <Wire.h>
#include <math.h>    
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "temperatureReporter.h"

#define VERSION "25.05.17.0"  //remember to update this after every change! YY.MM.DD.REV

U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE, 6, 5); // special tiny screen
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
OneWire oneWire(TEMPERATURE_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
 
// These are the settings that get stored in EEPROM.  They are all in one struct which
// makes it easier to store and retrieve.
typedef struct 
  {
  unsigned int validConfig=0; 
  char ssid[SSID_SIZE] = "";
  char wifiPassword[PASSWORD_SIZE] = "";
  char mqttBrokerAddress[ADDRESS_SIZE]=""; //default
  int mqttBrokerPort=1883;
  char mqttUsername[USERNAME_SIZE]="";
  char mqttPassword[PASSWORD_SIZE]="";
  char mqttTopicRoot[MQTT_TOPIC_SIZE]="";
  char mqttClientId[MQTT_CLIENTID_SIZE]=""; //will be the same across reboots
  bool debug=true;
  char address[ADDRESS_SIZE]=""; //static address for this device
  char netmask[ADDRESS_SIZE]=""; //size of network
  int32_t reportInterval=DEFAULT_STATUS_REPORT_INTERVAL; //How many seconds between status reports
  char tankSensorAddress[TEMPERATURE_SENSOR_ADDRESS_SIZE]; //address of the DS18S20 temperature sensor for tank
  char returnSensorAddress[TEMPERATURE_SENSOR_ADDRESS_SIZE]; //address of the DS18S20 temperature sensor for return line
  uint16_t pumpRunThreshold=DEFAULT_RUN_THRESHOLD; //analog reading above this means the pump is running
  uint16_t blowerRunThreshold=DEFAULT_RUN_THRESHOLD; //analog reading above this means the blower is running
  } conf;
conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

// I need to read these analog values before turning on the WiFi because the WiFi interferes with ADC2 (A3)
typedef struct
  {
  uint16_t rainSensorValue; //raw value from the rain sensor
  uint16_t batteryVoltage; //in millivolts
  uint16_t solarPanelVoltage; //in millivolts
  uint16_t vcc; //the voltage powering the ESP32 in millivolts
  } voltages;
voltages readings;


///////////// Global variables /////////////
IPAddress ip;
IPAddress mask;
String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed
float tankTemperatureC=-127.0; //latest tank temperature reading
float returnTemperatureC=-127.0; //latest return line temperature reading
bool blowerState=false; //is the blower currently running
bool pumpState=false; //is the pump currently running
int16_t pumpSwing=0; //the AC voltage swing for the pump
int16_t blowerSwing=0; //the AC voltage swing for the blower
ulong displayOffTimeMillis = 0; //time to turn off the display
//////////////////////////////////////////


//Generate an MQTT client ID.  This should not be necessary very often
char* generateMqttClientId(char* mqttId)
  {
  strcpy(mqttId,MQTT_CLIENT_ID_ROOT);
  strcat(mqttId, String(random(0xffff), HEX).c_str());
  if (settings.debug)
    {
    Serial.print("New MQTT userid is ");
    Serial.println(mqttId);
    }
  return mqttId;
  }

float ctof(float c)
  {
  return c * 9.0 / 5.0 + 32.0;
  }

/* Flips the display to be black-on-white or vice versa */
void flipDisplay()
  {
  static bool flipped=false;
  if (settings.debug)
    {
    Serial.println("Flipping display colors");
    }
  if (flipped)
    {
    flipped=false;
    u8g2.setBitmapMode(0); // Normal mode
    u8g2.sendF("c", 0x0a6); // 0xa6 is the hardware command for 'Normal Display'
    }
  else
    { 
    flipped=true;
    u8g2.setBitmapMode(1); // This flips the logic of the entire panel
    u8g2.sendF("c", 0x0a7); // 0xa7 is the hardware command for 'Inverse Display'
    }
  }

uint8_t nybble(char c)
  {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
  }

/* Convert the HR address to be machine-friendly */
void hexToBytes(const char* hex, uint8_t* bytes)
  {
  for (int i = 0; i < 8; i++)
    {
    // Take two characters from the string and convert to one byte
    char high = hex[i * 2];
    char low = hex[i * 2 + 1];
    
    // Use a standard helper to convert hex char to nibble
    bytes[i] = (nybble(high) << 4) | nybble(low);
    }
  }

// Determine if the pump or blower is running based on AC voltage swing
bool isRunning(int pin)
  {
  long totalSwing = 0;
  int iterations = 5; 

  for (int j = 0; j < iterations; j++)
    {
    int minVal = 4095;
    int maxVal = 0;
    
    // 1. Discard Read: Clears the "Ghost" charge from the previous pin
    analogRead(pin);
    delayMicroseconds(100); 

    unsigned long startTime = millis();
    while (millis() - startTime < 20) 
      {
      int val = analogRead(pin);
      if (val < minVal) minVal = val;
      if (val > maxVal) maxVal = val;
      }
    totalSwing += (maxVal - minVal);
    }

  int averageSwing = totalSwing / iterations;

  if (pin == PUMP_PIN) pumpSwing = averageSwing;
  if (pin == BLOWER_PIN) blowerSwing = averageSwing;

  bool running = false;
  if (pin == PUMP_PIN)
    {
    running=averageSwing > settings.pumpRunThreshold;
    }
  else if (pin == BLOWER_PIN)
    {
    running=averageSwing > settings.blowerRunThreshold;
    }

  return running;
  }



// Read temperature from a specific DS18S20 temperature sensor on the specified port
// portNumber: GPIO pin number where the DS18S20 sensor is connected
// sensorAddress: pointer to uint8_t array containing the 8-byte sensor address
// Returns: Temperature in Celsius as a float
float readDS18S20Temperature(uint8_t* sensorAddress)
  { 
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(sensorAddress);
  
  return tempC;
  }

void initializeSettings()
  {
  settings.validConfig=0; 
  strcpy(settings.ssid,"");
  strcpy(settings.wifiPassword,"");
  strcpy(settings.mqttBrokerAddress,""); //default
  settings.mqttBrokerPort=1883;
  strcpy(settings.mqttUsername,"");
  strcpy(settings.mqttPassword,"");
  strcpy(settings.mqttTopicRoot,"");
  strcpy(settings.tankSensorAddress,"");
  strcpy(settings.returnSensorAddress,"");
  settings.pumpRunThreshold=DEFAULT_RUN_THRESHOLD;
  settings.blowerRunThreshold=DEFAULT_RUN_THRESHOLD;
  settings.debug=true;
  strcpy(settings.address,"");
  strcpy(settings.netmask,"255.255.255.0");
  settings.reportInterval=DEFAULT_STATUS_REPORT_INTERVAL;
  generateMqttClientId(settings.mqttClientId);
  }


/* Send the settings to the serial port */
void showSettings()
  {
  Serial.print("broker=<MQTT broker host name or address> (");
  Serial.print(settings.mqttBrokerAddress);
  Serial.println(")");
  Serial.print("port=<port number>   (");
  Serial.print(settings.mqttBrokerPort);
  Serial.println(")");
  Serial.print("topicroot=<topic root> (");
  Serial.print(settings.mqttTopicRoot);
  Serial.println(")  Note: must end with \"/\"");  
  Serial.print("user=<mqtt user> (");
  Serial.print(settings.mqttUsername);
  Serial.println(")");
  Serial.print("pass=<mqtt password> (");
  Serial.print(settings.mqttPassword);
  Serial.println(")");
  Serial.print("ssid=<wifi ssid> (");
  Serial.print(settings.ssid);
  Serial.println(")");
  Serial.print("wifipass=<wifi password> (");
  Serial.print(settings.wifiPassword);
  Serial.println(")");
  Serial.print("address=<Static IP address if so desired> (");
  Serial.print(settings.address);
  Serial.println(")");
  Serial.print("netmask=<Network mask to be used with static IP> (");
  Serial.print(settings.netmask);
  Serial.println(")");
  Serial.print("tankSensorAddress=<Tank temperature sensor address> (");
  Serial.print(settings.tankSensorAddress);
  Serial.println(")");
  Serial.print("returnSensorAddress=<Return line temperature sensor address> (");
  Serial.print(settings.returnSensorAddress);
  Serial.println(")");
  Serial.print("pumpRunThreshold=<Analog threshold to detect pump running> (");
  Serial.print(settings.pumpRunThreshold);
  Serial.println(")");
  Serial.print("blowerRunThreshold=<Analog threshold to detect blower running> (");
  Serial.print(settings.blowerRunThreshold);
  Serial.println(")");
  Serial.print("debug=<1|0> (");
  Serial.print(settings.debug);
  Serial.println(")");
  Serial.print("reportinterval=<seconds between regular status reports>   (");
  Serial.print(settings.reportInterval);
  Serial.println(")");
  
  Serial.print("MQTT Client ID is ");
  Serial.println(settings.mqttClientId);
  Serial.print("Address is ");
  Serial.println(wifiClient.localIP());

  Serial.println("\n*** Use \"resetmqttid=yes\" to reset all settings  ***");
  Serial.println("*** Use \"factorydefaults=yes\" to reset all settings  ***\n");
  
  Serial.print("\nSettings are ");
  Serial.println(settingsAreValid?"valid.":"incomplete.");
  } 

/*
 * Check for configuration input via the serial port.  Return a null string 
 * if no input is available or return the complete line otherwise.
 */
String getConfigCommand()
  {
  if (commandComplete) 
    {
    Serial.println(commandString);
    String newCommand=commandString;
    if (newCommand.length()==0)
      newCommand='\n'; //to show available commands

    commandString = "";
    commandComplete = false;
    return newCommand;
    }
  else return "";
  }

/* Convert a string to lowercase */
void stringToLower(char* str)
  {
  if (str == NULL) 
    {
    return;
    }

  for (int i = 0; str[i] != '\0'; i++)
    {
    str[i] = (char)tolower((unsigned char)str[i]);
    }
  }


bool processCommand(String cmd)
  {
  bool commandFound=true; //saves a lot of code
  char nme[MAX_COMMAND_SIZE]; //shouldn't get any commands larger than this
  const char *str=cmd.c_str();
  char *val=NULL;
  char *nme_t=strtok((char *)str,"=");
  if (strlen(nme_t)<MAX_COMMAND_SIZE)
    {
    strcpy(nme,nme_t);//Don't modify c_str() pointers
    if (nme_t!=NULL)
      val=strtok(NULL,"=");
    else
      strcpy(nme,"\n"); 
    
    if (nme[0]=='\n' || nme[0]=='\r' || nme[0]=='\0') //a single cr means show current settings
      {
      showSettings();
      commandFound=false; //command not found
      }
    else
      {
      //Get rid of the carriage return
      if (val!=NULL && strlen(val)>0 && val[strlen(val)-1]==13)
        val[strlen(val)-1]=0; 

      stringToLower(nme); //make command name lowercase for easier matching

      if (val!=NULL)
        {
        if (strcmp(val,"NULL")==0) //to nullify a value, you have to really mean it
          {
          strcpy(val,"");
          }
        
        if (strcmp(nme,"broker")==0)
          {
          strcpy(settings.mqttBrokerAddress,val);
          saveSettings();
          }
        else if (strcmp(nme,"port")==0)
          {
          if (!val)
            strcpy(val,"0");
          settings.mqttBrokerPort=atoi(val);
          saveSettings();
          }
        else if (strcmp(nme,"topicroot")==0)
          {
          strcpy(settings.mqttTopicRoot,val);
          if (val[strlen(val)-1] !='/') // must end with a /
            {
            strcat(settings.mqttTopicRoot,"/");
            }
          saveSettings();
          }
        else if (strcmp(nme,"user")==0)
          {
          strcpy(settings.mqttUsername,val);
          saveSettings();
          }
        else if (strcmp(nme,"pass")==0)
          {
          strcpy(settings.mqttPassword,val);
          saveSettings();
          }
        else if (strcmp(nme,"ssid")==0)
          {
          strcpy(settings.ssid,val);
          saveSettings();
          }
        else if (strcmp(nme,"wifipass")==0)
          {
          strcpy(settings.wifiPassword,val);
          saveSettings();
          }
        else if (strcmp(nme,"address")==0)
          {
          strcpy(settings.address,val);
          saveSettings();
          }
        else if (strcmp(nme,"tanksensoraddress")==0)
          {
          strcpy(settings.tankSensorAddress,val);
          saveSettings();
          }
        else if (strcmp(nme,"blowerrunthreshold")==0)
          {
          if (!val)
            strcpy(val,"0");
          settings.blowerRunThreshold=atoi(val);
          saveSettings();
          }
        else if (strcmp(nme,"pumprunthreshold")==0)
          {
          if (!val)
            strcpy(val,"0");
          settings.pumpRunThreshold=atoi(val);
          saveSettings();
          }
        else if (strcmp(nme,"returnsensoraddress")==0)
          {
          strcpy(settings.returnSensorAddress,val);
          saveSettings();
          }
        else if (strcmp(nme,"netmask")==0)
          {
          strcpy(settings.netmask,val);
          saveSettings();
          }
        else if (strcmp(nme,"debug")==0)
          {
          if (!val)
            strcpy(val,"0");
          settings.debug=atoi(val)==1?true:false;
          saveSettings();
          }
        else if (strcmp(nme,"reportinterval")==0)
          {
          if (!val)
            strcpy(val,"0");
          settings.reportInterval=atoi(val);
          saveSettings();
          }

        else if ((strcmp(nme,"resetmqttid")==0)&& (strcmp(val,"yes")==0))
          {
          generateMqttClientId(settings.mqttClientId);
          saveSettings();
          }
        else if ((strcmp(nme,"factorydefaults")==0) && (strcmp(val,"yes")==0)) //reset all eeprom settings
          {
          Serial.println("\n*********************** Resetting EEPROM Values ************************");
          initializeSettings();
          saveSettings();
          delay(2000);
          ESP.restart();
          }
        else
          {
          showSettings();
          commandFound=false; //command not found
          }
        }
      }
    }
  else
    {
    Serial.print("Incoming command too large: ");
    Serial.println(nme_t);
    commandFound=false;
    }

  return commandFound;
  }

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void incomingSerialData() 
  {
  static bool lastCR = false; 
    {
    char inChar = (char)Serial.read(); // get the new byte
    Serial.print(inChar); //echo it back to the terminal

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it 
    if (inChar == '\n' || inChar == '\r') 
      {
      if (lastCR)     //some serial ports send both CR and LF, We want to ignore the second one
        lastCR=false;
      else
        {
        lastCR=true;
        commandComplete = true;
        }
      }
    else
      {
      lastCR=false; //in case only one of \r and \n is sent
      // add it to the inputString 
      commandString += inChar;
      }
    }
  }

void checkForCommand()
  {
  if (Serial.available())
    {
    incomingSerialData();
    String cmd=getConfigCommand();
    if (cmd.length()>0)
      {
      yield();
      processCommand(cmd);
      }
    }
  }

/* Read and return the battery voltage in millivolts */
uint32_t getBattery()
  {
  #define R1 470.0f // resistor values for the voltage divider
  #define R2 470.0f
  #define ADC_MAX_VALUE 4095.0f // 12-bit ADC

  // Fraction of battery voltage seen by ADC (0.5 for R1=R2)
  #define BATTERY_DIVIDER_RATIO (R2/(R1+R2)) 

  int result = analogRead(BATTERY_PIN_A3); // read the battery divider value

  // 1. Calculate the voltage at the divider tap (rawToMillivolts) using float math
  // (Result / ADC_MAX_VALUE) * ADC_MAX_VOLTAGE_MV
  float rawToMillivolts = ((float)result / ADC_MAX_VALUE) * ADC_MAX_VOLTAGE_MV; 

  // 2. Calculate the actual battery voltage (bt)
  // Voltage_Battery = Voltage_Tap / BATTERY_DIVIDER_RATIO (which is 0.5)
  float bt_float = rawToMillivolts / BATTERY_DIVIDER_RATIO; 
  
  // The final result should be returned as an integer (uint32_t), so cast it.
  uint32_t bt = (uint32_t)bt_float;

  Serial.print("\n**************** Raw Battery:  ");
  Serial.println(result);
  
  Serial.print("\n**************** Mapped Battery:  ");
  Serial.println(bt);
  return bt;
  }

/************************
 * Do the MQTT thing
 ************************/
bool report()
  {
  char topic[MQTT_TOPIC_SIZE+9];
  char reading[18];
  bool ok=true;

  //publish the tank sensor reading
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_TANK);
  sprintf(reading,"%.1f",ctof(tankTemperatureC)); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the return line sensor reading
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_RETURN);
  sprintf(reading,"%.1f",ctof(returnTemperatureC)); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the blower status
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_BLOWER);
  sprintf(reading,"%s",blowerState?"ON":"OFF"); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the pump status
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_PUMP);
  sprintf(reading,"%s",pumpState?"ON":"OFF"); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the pump voltage swing (diagnostic)
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_PUMP_SWING);
  sprintf(reading,"%d",pumpSwing); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the blower voltage swing (diagnostic)
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_BLOWER_SWING);
  sprintf(reading,"%d",blowerSwing); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  //publish the radio strength reading while we're at it
  strcpy(topic,settings.mqttTopicRoot);
  strcat(topic,MQTT_TOPIC_RSSI);
  sprintf(reading,"%d",WiFi.RSSI()); 
  ok=ok & publish(topic,reading,true); //retain
  yield();

  if (settings.debug)
    {
    Serial.print("Publish ");
    Serial.println(ok?"OK":"Failed");
    }
  return ok;
  }


boolean publish(char* topic, const char* reading, boolean retain)
  {
  if (settings.debug)
    {
    Serial.print(topic);
    Serial.print(" ");
    Serial.println(reading);
    }
  boolean ok=false;
  connectToWiFi(); //just in case we're disconnected from WiFi
  reconnectToBroker(); //also just in case we're disconnected from the broker

  if (mqttClient.connected() && 
      settings.mqttTopicRoot &&
      WiFi.status()==WL_CONNECTED)
    {
    ok=mqttClient.publish(topic,reading,retain); 
    mqttClient.loop(); //check for incoming messages
    }
  else
    {
    Serial.print("Can't publish due to ");
    if (WiFi.status()!=WL_CONNECTED)
      Serial.println("no WiFi connection.");
    else if (!mqttClient.connected())
      Serial.println("not connected to broker.");
    }
  return ok;
  }



/**
 * Handler for incoming MQTT messages.  The payload is the command to perform. 
 * The MQTT message topic sent is the topic root plus the command.
 * Implemented commands are: 
 * MQTT_PAYLOAD_SETTINGS_COMMAND: sends a JSON payload of all user-specified settings
 * MQTT_PAYLOAD_REBOOT_COMMAND: Reboot the controller
 * MQTT_PAYLOAD_VERSION_COMMAND Show the version number
 * MQTT_PAYLOAD_STATUS_COMMAND Show the most recent flow values
 */
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length) 
  {
  if (settings.debug)
    {
    Serial.println("====================================> Callback works.");
    }
  boolean rebootScheduled=false; //so we can reboot after sending the reboot response
  char charbuf[MQTT_MAX_INCOMING_PAYLOAD_SIZE];
  if (length < MQTT_MAX_INCOMING_PAYLOAD_SIZE)
    {
    payload[length]='\0'; //this should have been done in the calling code, shouldn't have to do it here
    sprintf(charbuf,"%s",payload);
    const char* response;
    
    
    //if the command is MQTT_PAYLOAD_SETTINGS_COMMAND, send all of the settings
    if (strcmp(charbuf,MQTT_PAYLOAD_SETTINGS_COMMAND)==0)
      {
      char tempbuf[35]; //for converting numbers to strings
      char jsonStatus[JSON_STATUS_SIZE];
      
      strcpy(jsonStatus,"{");
      strcat(jsonStatus,"\"broker\":\"");
      strcat(jsonStatus,settings.mqttBrokerAddress);
      strcat(jsonStatus,"\", \"port\":");
      sprintf(tempbuf,"%d",settings.mqttBrokerPort);
      strcat(jsonStatus,tempbuf);
      strcat(jsonStatus,", \"topicroot\":\"");
      strcat(jsonStatus,settings.mqttTopicRoot);
      strcat(jsonStatus,"\", \"user\":\"");
      strcat(jsonStatus,settings.mqttUsername);
      strcat(jsonStatus,"\", \"pass\":\"");
      strcat(jsonStatus,settings.mqttPassword);
      strcat(jsonStatus,"\", \"ssid\":\"");
      strcat(jsonStatus,settings.ssid);
      strcat(jsonStatus,"\", \"wifipass\":\"");
      strcat(jsonStatus,settings.wifiPassword);
      strcat(jsonStatus,"\", \"mqttClientId\":\"");
      strcat(jsonStatus,settings.mqttClientId);
      strcat(jsonStatus,"\", \"address\":\"");
      strcat(jsonStatus,settings.address);
      strcat(jsonStatus,"\", \"netmask\":\"");
      strcat(jsonStatus,settings.netmask);
      strcat(jsonStatus,"\", \"tanksensoraddress\":\"");
      strcat(jsonStatus,settings.tankSensorAddress);
      strcat(jsonStatus,"\", \"pumpRunThreshold\":\"");
      sprintf(tempbuf,"%d",settings.pumpRunThreshold);
      strcat(jsonStatus,tempbuf);
      strcat(jsonStatus,"\", \"blowerRunThreshold\":\"");
      sprintf(tempbuf,"%d",settings.blowerRunThreshold);
      strcat(jsonStatus,tempbuf);
      strcat(jsonStatus,"\", \"returnsensoraddress\":\"");
      strcat(jsonStatus,settings.returnSensorAddress);
      strcat(jsonStatus,"\", \"debug\":\"");
      strcat(jsonStatus,settings.debug?"true":"false");
      strcat(jsonStatus,"\", \"reportinterval\":\"");
      sprintf(tempbuf,"%d",settings.reportInterval);
      strcat(jsonStatus,tempbuf);
      strcat(jsonStatus,"\", \"IPAddress\":\"");
      strcat(jsonStatus,wifiClient.localIP().toString().c_str());
      strcat(jsonStatus,"\"");
      strcat(jsonStatus,"}");
      response=jsonStatus;
      }
    else if (strcmp(charbuf,MQTT_PAYLOAD_VERSION_COMMAND)==0) //show the version number
      {
      char tmp[15];
      strcpy(tmp,VERSION);
      response=tmp;
      }
    else if (strcmp(charbuf,MQTT_PAYLOAD_REBOOT_COMMAND)==0) //reboot the controller
      {
      char tmp[10];
      strcpy(tmp,"REBOOTING");
      response=tmp;
      rebootScheduled=true;
      }
    else if (processCommand(charbuf))
      {
      response="OK";
      }
    else
      {
      char badCmd[18];
      strcpy(badCmd,"(empty)");
      response=badCmd;
      }
      
    char topic[MQTT_TOPIC_SIZE];
    strcpy(topic,settings.mqttTopicRoot);
    strcat(topic,charbuf); //the incoming command becomes the topic suffix

    if (!publish(topic,response,false)) //do not retain
      Serial.println("************ Failure when publishing status response!");
      
    delay(2000); //give publish time to complete
    
    if (rebootScheduled)
      {
      ESP.restart();
      }
    }
  else
    Serial.println("Incoming MQTT message too large.");
  }


/*
 * Reconnect to the MQTT broker
 */
void reconnectToBroker() 
  {
  if (strlen(settings.mqttBrokerAddress)>0)
    {
    if (WiFi.status() != WL_CONNECTED)
      {
      Serial.println("WiFi not ready, skipping MQTT connection");
      }
    else
      {
      // Loop until we're reconnected or we give up
      int tries=20;
      while (!mqttClient.connected() && tries-->0) 
        {
        Serial.print("Attempting MQTT connection...");

        mqttClient.setBufferSize(JSON_STATUS_SIZE); //default (256) isn't big enough
        mqttClient.setKeepAlive(120); //seconds
        mqttClient.setServer(settings.mqttBrokerAddress, settings.mqttBrokerPort);
        mqttClient.setCallback(incomingMqttHandler);
        yield();

        // Attempt to connect
        if (mqttClient.connect(settings.mqttClientId,settings.mqttUsername,settings.mqttPassword))
          {
          Serial.println("connected to MQTT broker.");

          //resubscribe to the incoming message topic
          char topic[MQTT_TOPIC_SIZE];
          strcpy(topic,settings.mqttTopicRoot);
          strcat(topic,MQTT_TOPIC_COMMAND_REQUEST);
          bool subgood=mqttClient.subscribe(topic);
          showSub(topic,subgood);
          }
        else 
          {
          Serial.print("failed, rc=");
          Serial.println(mqttClient.state());
          Serial.println("Will try again in a second");
          
          // Wait a second before retrying
          // In the meantime check for input in case something needs to be changed to make it work
        //  checkForCommand(); 
          yield();
          delay(1000);
          yield();
          }
        checkForCommand();
        }
      if (!mqttClient.connected())
        {
        Serial.println("Could not connect to MQTT broker.");
        }
      else
        mqttClient.loop(); //This has to happen every so often to check for incoming messages
      }
    }
  else if (settings.debug)
    {
    Serial.println("Broker address not set, ignoring MQTT");
    }
  }

void showSub(char* topic, bool subgood)
  {
  if (settings.debug)
    {
    Serial.print("++++++Subscribing to ");
    Serial.print(topic);
    Serial.print(":");
    Serial.println(subgood);
    }
  }

/**
 * @brief Checks if a C-style string (char array) contains only
 * alphanumeric characters (a-z, A-Z, 0-9).
 *
 * @param str The null-terminated char array to check.
 * @return true if all characters are alphanumeric or the string is empty.
 * @return false if any character is not alphanumeric.
 */
bool checkString(const char* str) 
  {
    if (str == nullptr) 
      {
      return false; // A null pointer is not a valid string
      }

    // Iterate through the string until the null terminator
    for (size_t i = 0; str[i] != '\0'; ++i) 
      {
      // Cast char to unsigned char to avoid potential issues with 
      // negative char values when passed to ctype functions.
      unsigned char ch = static_cast<unsigned char>(str[i]); // Cast once
      if (!isalnum(ch) && ch != '/' && ch != '.')
        {
        return false; // Found a non-alphanumeric character
        }
      }

  // If the loop completes, all characters were alphanumeric (or the string was empty)
  return true;
  }


/*
 * Check all of the strings in the settings.   If any of the
 * character strings in the settings fail the test, it's 
 * likely that the settings are corrupt. 
*/  
bool settingsSanityCheck()
  {
  return checkString(settings.ssid)
      && checkString(settings.wifiPassword)
      && checkString(settings.mqttBrokerAddress)
      && checkString(settings.mqttUsername)
      && checkString(settings.mqttPassword)
      && checkString(settings.mqttTopicRoot)
      && checkString(settings.mqttClientId)
      && checkString(settings.address)
      && checkString(settings.netmask)
      && checkString(settings.tankSensorAddress)
      && checkString(settings.returnSensorAddress)
      ;
  }

/*
 * Save the settings to EEPROM. Set the valid flag if everything is filled in.
 */
boolean saveSettings()
  {
  if (strlen(settings.ssid)>0 &&
      strlen(settings.wifiPassword)>0 &&
      // strlen(settings.mqttBrokerAddress)>0 &&
      // settings.mqttBrokerPort!=0 &&
      strlen(settings.mqttTopicRoot)>0 &&
      strlen(settings.mqttClientId)>0 &&
      settingsSanityCheck())
    {
    Serial.println("Settings deemed complete");
    settings.validConfig=VALID_SETTINGS_FLAG;
    settingsAreValid=true;
    }
  else
    {
    Serial.println("Settings still incomplete");
    settings.validConfig=0;
    settingsAreValid=false;
    }
    
  //The mqttClientId is not set by the user, but we need to make sure it's set  
  if (strlen(settings.mqttClientId)==0)
    {
    generateMqttClientId(settings.mqttClientId);
    }
    
  EEPROM.put(0,settings);
  if (settings.debug)
    Serial.println("Committing settings to eeprom");
  return EEPROM.commit();
  }


/*
*  Initialize the settings from eeprom and determine if they are valid
*/
void loadSettings()
  {
  EEPROM.get(0,settings);

  if (!settingsSanityCheck()) //if something is wildly off then don't run, allow setup
    {
    settings.validConfig=0;
    settingsAreValid=false;
    Serial.println(F("Settings are corrupt, marking invalid."));
    }
  else if (settings.validConfig==VALID_SETTINGS_FLAG)    //skip loading stuff if it's never been written
    {
    settingsAreValid=true;
    if (settings.debug)
      {
      Serial.println("\nLoaded configuration values from EEPROM");
      }
    }
  else
    {
    Serial.println("Skipping load from EEPROM, device not configured.");    
    settingsAreValid=false;
    }
    showSettings();
  }


 /*
 * If not connected to wifi, connect.
 */
void connectToWiFi()
  {
  if (settingsAreValid && WiFi.status() != WL_CONNECTED)
    {
    Serial.print("Attempting to connect to WPA SSID \"");
    Serial.print(settings.ssid);
    Serial.println("\"");

    WiFi.disconnect(true); // Completely reset Wi-Fi stack
    delay(100); // Small delay to ensure reset is applied
    WiFi.persistent(false); // Prevent saving to flash
    WiFi.mode(WIFI_STA); //station mode, we are only a client in the wifi world

    if (settingsAreValid)
      {      
      if (!ip.fromString(settings.address))
        {
        Serial.println("Static IP Address '"+String(settings.address)+"' is blank or not valid. Using dynamic addressing.");
        }
      else if (!mask.fromString(settings.netmask))
        {
        Serial.println("Static network mask "+String(settings.netmask)+" is not valid. Using default");
        mask.fromString("255.255.255.0");
        }
      }
    if (ip) //Use the static address
      {
      if (!WiFi.config(ip,ip,mask))
        {
        Serial.println("STA Failed to configure");
        }
      }

    unsigned long connectTimeout = millis() + WIFI_TIMEOUT_SECONDS*1000; // 30 second timeout
    WiFi.begin(settings.ssid, settings.wifiPassword);
    //delay(1000);
    unsigned long lastDotTime = millis(); // For printing dots without blocking
    while (WiFi.status() != WL_CONNECTED && millis() < connectTimeout) 
      {
      // Not yet connected
      if (millis() - lastDotTime > 500) // Print dot every 500ms, but don't block
        {
        Serial.print(".");
        lastDotTime = millis();
        yield();
        }
      checkForCommand(); // Check for input in case something needs to be changed to work
      yield();
      }

    if (WiFi.status() == WL_CONNECTED)
      {
      Serial.print("\nConnected to network with address ");
      Serial.println(WiFi.localIP());
      Serial.println();
      }
    else
      {
      Serial.println("\nFailed to connect to WiFi\n");
      }
    }
  }
 

void show(const char* text)
  {
  u8g2.clearBuffer();
  
  // Start at the top left
  int x = 4; //Need a little margin
  int y = 8; // Starting Y (based on font height)
  int lineHeight = 10; 
  
  char buffer[32]; // Temporary line buffer
  const char* p = text;
  const char* nextLine;
  
  while ((nextLine = strchr(p, '\n')) != NULL)
    {
    int len = nextLine - p;
    if (len > 31) len = 31;
    
    strncpy(buffer, p, len);
    buffer[len] = '\0';
    
    u8g2.drawStr(x, y, buffer);
    
    y += lineHeight;
    p = nextLine + 1;
    }
    
  // Draw the remaining text after the last \n
  u8g2.drawStr(x, y, p);
  
  u8g2.sendBuffer();
  }  

void readSensors()
  {
  uint8_t address[8]; 
  hexToBytes(settings.tankSensorAddress, address);
  tankTemperatureC = readDS18S20Temperature(address); // read the tank temperature 
  hexToBytes(settings.returnSensorAddress, address);
  returnTemperatureC = readDS18S20Temperature(address); // read the return line temperature 

  pumpState = isRunning(PUMP_PIN);
  blowerState = isRunning(BLOWER_PIN);
  }

void initSerial()
  {
  Serial.begin(115200);

  // Set the TX timeout to 0 milliseconds so that it will run without a serial port attached
  //Serial.setTxTimeoutMs(0); 

  // Now, all subsequent Serial.print() and Serial.println() calls 
  // should be non-blocking. If the buffer is full (i.e., the PC isn't listening), 
  // the data is dropped, and the program continues immediately.  
  
  delay(100); //give time for serial to start 
  Serial.println();
  Serial.println("Serial communications established.");
  delay(5000);
  commandString.reserve(200); // reserve 200 bytes of serial buffer space for incoming command string
  }

void initDisplay()
  {
  //Wire.begin(5, 6);
  u8g2.begin();
  u8g2.setContrast(10);    // reduce the brightness
  u8g2.setBusClock(400000); // 400kHz I2C
  u8g2.setFontRefHeightText(); // This tells u8g2 to use the font height for line spacing
//  u8g2.setFontPosTop(); // Set cursor at top of font rather than baseline
  u8g2.setFont(u8g2_font_5x7_tr); // set the font for the display
  show("Starting up...");
  }


void initSettings()
  {
  EEPROM.begin(sizeof(settings)); //fire up the eeprom section of flash
  loadSettings(); //set the values from eeprom 

  //show the MAC address
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (settings.mqttBrokerPort < 0) //then this must be the first powerup
    {
    Serial.println("\n*********************** Resetting All EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }
  }


void initPorts()
  {
  sensors.begin();

  pinMode(LED_BUILTIN, OUTPUT);   // LED will indicate active wifi connection
  digitalWrite(LED_BUILTIN, HIGH); // it's active low

  analogReadResolution(12);//  Set 12-bit resolution for all ADC reads

  // Set all ADC ranges to to 0-2.2V
  analogSetPinAttenuation(PUMP_PIN, ADC_11db); 
  analogSetPinAttenuation(BLOWER_PIN, ADC_11db); 
  }


void setup(void) 
  {
  initSerial();

  initSettings();
  
  initDisplay();

  initPorts();
  }

void loop(void) 
  {
  ulong now= millis();

  readSensors();// take a reading from all sensors

  /////////// Display stuff
 // display the new reading
  char buffer[80];
  char tempbuf[15];
  if (settingsAreValid) //always show readings for debugging
    {    
    sprintf(tempbuf, "Tank: %.1f\n", 
            ctof(tankTemperatureC)); 
    sprintf(buffer, tempbuf);
    
    sprintf(tempbuf, "Line: %.1f\n", 
            ctof(returnTemperatureC)); 
    sprintf(buffer + strlen(buffer), tempbuf);

    sprintf(tempbuf, "Pump: %s\n", 
            pumpState?"ON":"OFF");
    sprintf(buffer + strlen(buffer), tempbuf);

    sprintf(tempbuf, "Blower: %s\n", 
            blowerState?"ON":"OFF"); 
    sprintf(buffer + strlen(buffer), tempbuf);
    }  
  else
    sprintf(buffer, "Settings\nInvalid");

  show(buffer); // display the reading on the OLED screen
    
  //////////// Reporting stuff
  if (settingsAreValid)
    {
    if (report()) //try to report the reading
      {
      Serial.print("Report complete.\n");
      }
    else
      {
      Serial.print("Report failed.\n");
      }
    }
  else
    {
    Serial.println("Settings not valid, cannot report.");
    while(settingsAreValid==false) //wait here until settings are fixed
      {
      checkForCommand(); //check for input in case something needs to be changed to make it work
      delay(100); //don't burn up the CPU
      }
    Serial.println("Settings fixed.");
    }
  
  while(millis()-now < settings.reportInterval * 1000UL)
    {
    if (millis()%1800000 <100) //every 30 minutes
      flipDisplay(); //flip the display colors for burn-in reduction
    
    if (WiFi.status() != WL_CONNECTED 
        || !mqttClient.connected())
      digitalWrite(LED_BUILTIN, HIGH); //turn off the LED
    else
      digitalWrite(LED_BUILTIN, LOW); //turn on the LED

    mqttClient.loop(); //keep the mqtt connection alive
    checkForCommand(); //check for input in case something needs to be changed to make it work
    delay(100); //don't burn up the CPU
    }
  }

