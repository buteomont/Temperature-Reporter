//The pin definitions below are for the Seeed Studio XIAO ESP32C3 board
//They seem to be different from the ESP32 Dev board
//#define SENSOR_POWER_PIN D4 //This pin powers the sensor on and off
#define DISPLAY_TIMEOUT 30000 //milliseconds to keep the display on after last use
#define SENSOR_PIN_A0 2 //The sensor is connected to ADC0
#define TEMPERATURE_SENSOR_PIN 3 //The DS18S20 temperature sensors are connected to GPIO3
#define BATTERY_PIN_A3 5 //The raw battery voltage divider is connected to ADC3
#define PUMP_PIN 0
#define BLOWER_PIN 1
#define ADC_MAX_VOLTAGE_MV 3300 //ESP32 ADC reference voltage

#define DEFAULT_STATUS_REPORT_INTERVAL 5 //60*60 //maximum seconds between dry time status reports
#define VALID_SETTINGS_FLAG 0xDAB0
#define SSID_SIZE 50
#define PASSWORD_SIZE 50
#define ADDRESS_SIZE 30
#define USERNAME_SIZE 50
#define TEMPERATURE_SENSOR_ADDRESS_SIZE 17 //The DS18S20 temperature sensor address size in characters (16 + null)
#define MAX_COMMAND_SIZE 50 // incoming command names are all smaller than this
#define MQTT_CLIENTID_SIZE 25
#define MQTT_TOPIC_SIZE 150
#define MQTT_TOPIC_SUFFIX_SIZE 15
#define MQTT_TOPIC_BLOWER "blower" //topic suffix for the blower status
#define MQTT_TOPIC_PUMP "pump" //topic suffix for the pump status
#define MQTT_TOPIC_PUMP_SWING "swing/pump" //topic suffix for the pump AC voltage swing
#define MQTT_TOPIC_BLOWER_SWING "swing/blower" //topic suffix for the blower AC voltage swing
#define MQTT_TOPIC_TANK "tankTemp" //topic suffix for the tank temperature
#define MQTT_TOPIC_RETURN "returnTemp" //topic suffix for the return line temperature
#define MQTT_TOPIC_RSSI "rssi"
#define MQTT_CLIENT_ID_ROOT "boiler"
#define MQTT_TOPIC_COMMAND_REQUEST "command"
#define MQTT_PAYLOAD_SETTINGS_COMMAND "settings" //show all user accessable settings
#define MQTT_PAYLOAD_REBOOT_COMMAND "reboot" //reboot the controller
#define MQTT_PAYLOAD_VERSION_COMMAND "version" //show the version number
#define MQTT_PAYLOAD_STATUS_COMMAND "status" //show the most recent flow values
#define MQTT_MAX_INCOMING_PAYLOAD_SIZE 100 //incoming MQTT message should never be this big
#define JSON_STATUS_SIZE SSID_SIZE+PASSWORD_SIZE+USERNAME_SIZE+MQTT_TOPIC_SIZE+ADDRESS_SIZE+(MQTT_TOPIC_SUFFIX_SIZE*2)+250 //+250 for associated field names, etc
#define WIFI_TIMEOUT_SECONDS 30 // give up on wifi after this long
#define STANDALONE_SSID "rainSensor" //SSID to use when in soft AP mode
#define STAY_AWAKE_SECONDS 30 //stay awake for this many extra seconds when receiving serial commands
#define DEFAULT_RUN_THRESHOLD 80 // A swing of 100 is roughly 0.7A.

// Function prototypes (generated from src/main.cpp)
float readDS18S20Temperature(uint8_t* sensorAddress);
char* generateMqttClientId(char* mqttId);
void initializeSettings();
void showSettings();
String getConfigCommand();
bool processCommand(String cmd);
void incomingSerialData();
void checkForCommand();
bool report(uint16_t value);
boolean publish(char* topic, const char* reading, boolean retain);
void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length);
void reconnectToBroker();
void showSub(char* topic, bool subgood);
bool checkString(const char* str);
bool settingsSanityCheck();
boolean saveSettings();
void loadSettings();
void connectToWiFi();
void show(const char *s);
//voltages readSensors();
void initSerial();
void initDisplay();
void initSettings();
void initPorts();
void setup(void);
void loop(void);

