// Sensors
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SGP30.h>
#include <DHT.h>

// Networking
#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>

// ESP32
#include <esp32-hal-adc.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <EEPROM.h>

// Utils
#include <Arduino_JSON.h>

// MQTT struct to hold payload and topic to publish
struct MQTTMessage {
  JSONVar json;
  char* topic;
};

// SGP30 baselines struct stored in eeprom 
struct SGP30Baselines
{
    uint16_t eCO2;
    uint16_t TVOC;
};

// Sensor data structs
struct DHT22Data {
  float temperature;
  float humidity;
  float heatIndex;
  float temperaturef;
  float heatIndexf;
};

struct TSL2561Data {
  uint16_t luminosity;
  uint16_t broadband;
  uint16_t infrared;
  uint16_t uvIndex;
};

struct SGP30Data {
  uint16_t tvoc;
  uint16_t tvoc_base;
  uint16_t co2;
  uint16_t co2_base;
  uint16_t h2;
  uint16_t ethanol;
};

struct BME280Data {
  float temperature;
  float humidity;
  float preassure;
  float approximateAltitude;
};

//--- Globals ---//
// Wifi
const char* ssid = "<SSID>";
const char* password = "<PASSWORD>";

WebServer server(80);
// MQTT client attached to wificlient
WiFiClient espClient;

/*
 * MQTT callback handler
 */
void mqtt_callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}

// MQTT Broker IP address
IPAddress mqtt_server(0, 0, 0, 0);
const char* MQTT_UID = "esp32dev";

// ID for MQTT connection
uint16_t MQTT_ID = 0;
bool register_MQTT_sensors;

PubSubClient client(mqtt_server, 1883, mqtt_callback, espClient);

// SGP30 
Adafruit_SGP30 SGP;
SGP30Baselines SGP30_baselines;

// DHT22 sensor
#define DHTTYPE DHT22
int DHTPIN = 4;
DHT dht(DHTPIN, DHTTYPE);

/*
 * https://github.com/adafruit/Adafruit_SGP30/blob/master/examples/sgp30test/sgp30test.ino
 * 
 * return absolute humidity [mg/m^3] with approximation formula
 * @param temperature [°C]
 * @param humidity [%RH]
 */
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

// TSL2561 Luminosity Sensor
/* I2C
 *  
 * The ADDR pin can be used if you have an i2c address conflict, to change the address
 * Connect it to ground to set the address to 0x29, connect it to 3.3V (vcc*) to set the address to 0x49 
 * or leave it floating (unconnected) to use address 0x39.
 * 
 * The INT pin is an ouput from the sensor used when you have the sensor configured to signal when the light level has changed. 
 * We don't have that code written in this tutorial so you don't have to use it. 
 * If you do end up using it, use a 10K-100K pullup from INT to 3.3V (vcc)
 * 
 */
#define TSL2561_SEL_LOW 0x29
#define TSL2561_ADDR_FLOAT 0x39
#define TSL2561_HIGH 0x49
#define TSL2561_UNIQUE_ID 12345

// UV sensor
uint16_t UVPIN = 33;

// The address will be different depending on whether you leave
// the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, TSL2561_UNIQUE_ID);

// BME289
#define BME289_ADDR_FLOAT 0x76
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

//--- End globals ---//

/* 
 * Device setup, run once
 */
void setup(void) {
  Serial.begin(115200);
  
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Wait for serial
  while (!Serial) delay(100);

  // Init EEPROM
  EEPROM.begin(255);
    
  // Establsih wifi connection
  //WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Set up MTQQ and register sensors if successfully connected
  register_MQTT_sensors = true;
  connect_MQTT();
  
  // Start HTTP server and set handlers
  
  // Temperature
  server.on("/DHT22", handleDHT22);
  
  // Luminousity
  server.on("/luminosity", handleTSL2561);

  server.onNotFound(handleNotFound);
  server.begin();
  
  Serial.println("HTTP server started"); 
  
  // Start DHT22 sensor
  dht.begin();
  
  // Start and configure TSL2561 Luminousity sensor
  if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  configure_tls25621_sensor();
 
  // Configure SGP32 sensor
  configure_sgp30_sensor();

  // Configure BME280 sensor
  configure_bme280_sensor();
}

/*
 * Register sensors to homeassistant with MQTT integration
 */
void register_MQTT_sensors_to_homeassistant(){
  Serial.println("Registering Home assistant MQTT sensors");
  // Create Home Assistant sensors
      
  // Temperature
  JSONVar temperature = new JSONVar();
  temperature["unique_id"] = "esp32dev-temperature";
  temperature["name"] = "Temperature";
  temperature["state_topic"] = "homeassistant/sensor/esp32dev/state";
  temperature["unit_of_measurement"] = "°C";
  temperature["value_template"] = "{{ value_json.temperature | round(1) }}";
  
  // Humidity
  JSONVar humidity = new JSONVar();
  humidity["unique_id"] = "esp32dev-humidity";
  humidity["name"] = "Temperature";
  humidity["state_topic"] = "homeassistant/sensor/esp32dev/state";
  humidity["unit_of_measurement"] = "%";
  humidity["value_template"] = "{{ value_json.temperature | round(1) }}";
  
  // Feels like
  JSONVar feelsLike = new JSONVar();
  feelsLike["unique_id"] = "esp32dev-feelslike";
  feelsLike["name"] = "Feels like";
  feelsLike["state_topic"] = "homeassistant/sensor/esp32dev/state";
  feelsLike["unit_of_measurement"] = "°C";
  feelsLike["value_template"] = "{{ value_json.heatIndex | round(1) }}";  
  
  // Luminosity
  JSONVar luminosity = new JSONVar();
  luminosity["unique_id"] = "esp32dev-illuminance";
  luminosity["name"] = "Luminosity";
  luminosity["state_topic"] = "homeassistant/sensor/esp32dev/state";
  luminosity["unit_of_measurement"] = "lux";
  luminosity["value_template"] = "{{ value_json.luminosity }}";
  
  // UV light
  JSONVar uv = new JSONVar();
  uv["unique_id"] = "esp32dev-uvIndex";
  uv["name"] = "UV light";
  uv["state_topic"] = "homeassistant/sensor/esp32dev/state";
  uv["unit_of_measurement"] = "";
  uv["value_template"] = "{{ value_json.uv }}"; 
  
  // TVOC
  JSONVar tvoc = new JSONVar();
  tvoc["unique_id"] = "esp32dev-tvoc";
  tvoc["name"] = "TVOC";
  tvoc["state_topic"] = "homeassistant/sensor/esp32dev/SGP30/state";
  tvoc["unit_of_measurement"] = "ppb";
  tvoc["value_template"] = "{{ value_json.tvoc }}";  
    
  // CO2
  JSONVar co2 = new JSONVar();
  co2["unique_id"] = "esp32dev-co2";
  co2["name"] = "eCO2";
  co2["state_topic"] = "homeassistant/sensor/esp32dev/SGP30/state";
  co2["unit_of_measurement"] = "ppm";
  co2["value_template"] = "{{ value_json.co2 }}"; 
  
  // H2
  JSONVar h2 = new JSONVar();
  h2["unique_id"] = "esp32dev-h2";
  h2["name"] = "H2";
  h2["state_topic"] = "homeassistant/sensor/esp32dev/SGP30/state";
  h2["unit_of_measurement"] = "ppm";
  h2["value_template"] = "{{ value_json.h2 }}";  
  
  // Ethanol
  JSONVar ethanol = new JSONVar();
  ethanol["unique_id"] = "esp32dev-ethanol";
  ethanol["name"] = "Ethanol";
  ethanol["state_topic"] = "homeassistant/sensor/esp32dev/SGP30/state";
  ethanol["unit_of_measurement"] = "ppm";
  ethanol["value_template"] = "{{ value_json.ethanol }}";  
  
  // BME280 Temperature
  JSONVar bme280temperature = new JSONVar();
  bme280temperature["unique_id"] = "esp32dev-bmetemp";
  bme280temperature["name"] = "BME280 Temperature";
  bme280temperature["state_topic"] = "homeassistant/sensor/esp32dev/BME/state";
  bme280temperature["unit_of_measurement"] = "°C";
  bme280temperature["value_template"] = "{{ value_json.bme280_temp | round(1) }}";
  
  // BME280 Humidity
  JSONVar bme280Humidity = new JSONVar();
  bme280Humidity["unique_id"] = "esp32dev-bmehumidity";
  bme280Humidity["name"] = "BME280 Humidity";
  bme280Humidity["state_topic"] = "homeassistant/sensor/esp32dev/BME/state";
  bme280Humidity["unit_of_measurement"] = "%";
  bme280Humidity["value_template"] = "{{ value_json.bme280_humidity | round(1) }}";
  
  // BME280 Preassure
  JSONVar bme280Preassure = new JSONVar();
  bme280Preassure["unique_id"] = "esp32dev-bmepres";
  bme280Preassure["name"] = "BME280 Preassure";
  bme280Preassure["state_topic"] = "homeassistant/sensor/esp32dev/BME/state";
  bme280Preassure["unit_of_measurement"] = "hPa";
  bme280Preassure["value_template"] = "{{ value_json.bme280_pres | round(2) }}";
  
  // BME280 Approximate altitude
  JSONVar bme280Altitude= new JSONVar();
  bme280Altitude["unique_id"] = "esp32dev-bmealtitude";
  bme280Altitude["name"] = "BME280 Altitude";
  bme280Altitude["state_topic"] = "homeassistant/sensor/esp32dev/BME/state";
  bme280Altitude["unit_of_measurement"] = "m";
  bme280Altitude["value_template"] = "{{ value_json.bme280_altitude | round(2) }}";

  // Publish sensors to MQTT
  byte sensorCount = 13;
  MQTTMessage sensors[] = {
    {temperature,       "homeassistant/sensor/esp32dev/Temperature/config"},
    {humidity,          "homeassistant/sensor/esp32dev/Humidity/config"},
    {feelsLike,         "homeassistant/sensor/esp32dev/Heatindex/config"},
    {luminosity,        "homeassistant/sensor/esp32dev/Luminosity/config"},
    {uv,                "homeassistant/sensor/esp32dev/UvIndex/config"},
    {tvoc,              "homeassistant/sensor/esp32dev/TVOC/config"},
    {co2,               "homeassistant/sensor/esp32dev/CO2/config"},
    {h2,                "homeassistant/sensor/esp32dev/H2/config"},
    {ethanol,           "homeassistant/sensor/esp32dev/Ethanol/config"},
    {bme280temperature, "homeassistant/sensor/esp32dev/BMETemp/config"},
    {bme280Humidity,    "homeassistant/sensor/esp32dev/BMEHum/config"},
    {bme280Preassure,   "homeassistant/sensor/esp32dev/BMEPres/config"},
    {bme280Altitude,    "homeassistant/sensor/esp32dev/BMEAlt/config"},
  };
  
  for (byte i = 0; i < sensorCount; i++) {
    MQTTMessage sensor = sensors[i];
    String jsonString = JSON.stringify(sensor.json);
    byte stringLength = jsonString.length() + 1;
    char payload[stringLength];
    jsonString.toCharArray(payload, stringLength);
    Serial.println(sensor.topic);
    Serial.println(payload);
    client.publish(sensor.topic, payload);
  }
  
  Serial.println("Home assistant sensors registered.");
}

/* 
 *  Configures the gain and integration time for the TSL2561
 */ 
void configure_tls25621_sensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);          /* Auto-gain ... switches automatically between 1x and 16x */
 
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
  
  sensor_t sensor;
  tsl.getSensor(&sensor);
  
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  

  // Get a new sensor event 
  sensors_event_t event;
  tsl.getEvent(&event);
 
  // Display the results (light is measured in lux)
  if (event.light)
  {
    Serial.print(event.light); Serial.println(" lux");
  }
  else
  {
    // If event.light = 0 lux the sensor is probably saturated
    //   and no reliable data could be generated! 
    Serial.println("Sensor overload");
  }
}

/* 
 *  BME280 sensor configuration
 */
void configure_bme280_sensor() {
  unsigned status;
  status = bme.begin(BME289_ADDR_FLOAT);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
}

/* 
 *  SGP30 sensor configuration
 */
void configure_sgp30_sensor(){
   if (!SGP.begin()){
    Serial.println("Sensor not found");
    return;
  }
  
  // Initials to EEPROM
  //SGP30_baselines = {36455, 36704};
  //EEPROM.put(0, SGP30_baselines);
  //EEPROM.commit();
  
  // Read SGP30 baselines from EEPROM
  EEPROM.get(0, SGP30_baselines);
  
  // Serial
  Serial.print("Found SGP30 serial #");
  Serial.print(SGP.serialnumber[0], HEX);
  Serial.print(SGP.serialnumber[1], HEX);
  Serial.println(SGP.serialnumber[2], HEX);
  
  if (!SGP.IAQinit()){
    Serial.println("IAQ init failed");
  }

  // Read temperature and humidity from DHT22
  DHT22Data dht22 = read_dht();
  Serial.println("Setting SGP30 absolute humidity");
  Serial.print("°C: ");Serial.print(dht22.temperature);Serial.print(" RH%: ");Serial.println(dht22.humidity);
  SGP.setHumidity(getAbsoluteHumidity(dht22.temperature, dht22.humidity));
  
  // If you have a baseline measurement from 
  // before you can assign it to start, to 'self-calibrate'
  Serial.println("Setting SGP30 baselines");
  Serial.print("eCO2: ");Serial.print(SGP30_baselines.eCO2);Serial.print(" TVOC: ");Serial.println(SGP30_baselines.TVOC);
  SGP.setIAQBaseline(SGP30_baselines.eCO2, SGP30_baselines.TVOC);
}

/*
 * Read DHT22 sensor data and return data struct as JSON
 */
void handleDHT22() {
    DHT22Data dht_state = read_dht();
    // Create json object
    JSONVar json;
    // Temperature
    json["temperature"]["value"] = dht_state.temperature;
    json["temperature"]["unit"] = "°C";
    // Humidity
    json["humidity"]["value"] = dht_state.humidity;
    json["humidity"]["unit"] = "%";
    // Heat index - Temperature with humidity combined
    json["temperature"]["heatIndex"]["value"] = dht_state.heatIndex;
    json["temperature"]["heatIndex"]["unit"] = "°C";
    String jsonDoc = JSON.stringify(json);
    server.send(200, "text/json", jsonDoc);
}

/*
 * Read TSL2561 sensor data and return data struct as JSON
 */
void handleTSL2561() {
    TSL2561Data luminosity = read_luminosity();
    // Create json object
    JSONVar json;
    json["luminosity"]["value"] = luminosity.luminosity;
    json["luminosity"]["unit"] = "lux";
    json["broadband"]["raw"] = luminosity.broadband;
    json["broadband"]["value"] = luminosity.broadband / 4.6;
    json["broadband"]["unit"] = "uW/cm^2";
    // For the TSL2561, 6.3 raw counts corresponds to 1 uW/cm^2 (microwatt per square centimeter)
    json["infrared"]["raw"] = luminosity.infrared;
    json["infrared"]["value"] = luminosity.infrared / 6.3;
    json["infrared"]["unit"] = "uW/cm^2";
    // UV value
    json["uv"]["value"] = luminosity.uvIndex;
    String jsonDoc = JSON.stringify(json);
    server.send(200, "text/json", jsonDoc);
}

/*
 * 404 handler for web server
 */
void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  
  server.send(404, "text/plain", message);
}

/*
 * Read DHT22 sensor data and return data struct
 */
DHT22Data read_dht() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return {};
  }
  
  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  
  DHT22Data data = {h, t ,hic, f, hif};
  return data;
}

// Readings which after to reset baseline
uint16_t sgp30_readings = 0;

/*
 * Read SGP30 sensor data and return data struct
 */
SGP30Data read_sgp30() {
  // Read temperature and humidity from DHT22
  DHT22Data dht22 = read_dht();
  
  Serial.println("Setting SGP30 absolute humidity");
  Serial.print("°C: ");Serial.print(dht22.temperature);Serial.print(" RH%: ");Serial.println(dht22.humidity);
  
  SGP.setHumidity(getAbsoluteHumidity(dht22.temperature, dht22.humidity));

  // Measure
  if (!SGP.IAQmeasure()) {
    Serial.println("Measurement failed");
  }
  if (!SGP.IAQmeasureRaw()) {
    Serial.println("Raw Measurement failed");
  }
  
  // Readings
  uint16_t tvoc = SGP.TVOC;
  uint16_t co2 = SGP.eCO2;
  uint16_t h2 = SGP.rawH2;
  uint16_t ethanol = SGP.rawEthanol;
  
  // Recalibrate baseline once in 12 hours, sensor reading is once a minute
  sgp30_readings += 1;
  if (sgp30_readings > 60 * 12) {
    Serial.println("Reading SGP30 baselines");
    uint16_t TVOC_base;
    uint16_t eCO2_base;
    if (SGP.getIAQBaseline(&eCO2_base, &TVOC_base)) {
      Serial.print("Baseline values: eCO2: "); Serial.print(eCO2_base); Serial.print(" & TVOC: "); Serial.println(TVOC_base);
      // Save baselines to EEPROM
      SGP30_baselines.eCO2 = eCO2_base;
      SGP30_baselines.TVOC = TVOC_base;
      EEPROM.put(0, SGP30_baselines);
      EEPROM.commit();
    } else {
      Serial.println("Failed to read baselines");
    }
    
    sgp30_readings = 0;
  }
  
  SGP30Data data = {tvoc, SGP30_baselines.TVOC, co2, SGP30_baselines.eCO2, h2, ethanol};
  return data;
}

/*
 * Read BME280 sensor data and return data struct
 */
BME280Data read_bme280() {
  float preassure = bme.readPressure() / 100.0F;
  float humidity = bme.readHumidity();
  float temperature = bme.readTemperature();
  float approximateAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
  BME280Data data = {temperature, humidity, preassure, approximateAltitude};
  return data;
}

/*
 * Read TSL2561 sensor data and return data struct
 */
TSL2561Data read_luminosity(){
  // Get a new sensor event 
  sensors_event_t event;
  tsl.getEvent(&event);
  
  // Lux - visible light
  uint16_t broadband = 0; 
  
  // Infared light
  uint16_t infrared = 0;  
 
  // Populate broadband and infrared with the latest values 
  tsl.getLuminosity (&broadband, &infrared);
  
  // UV
  uint16_t sensorValue = analogRead(UVPIN);
  uint16_t sensorVoltage = (sensorValue / 4095) * 3.3;
  uint16_t uvVal = getUvIndex(sensorVoltage);

  TSL2561Data data = {event.light, broadband, infrared, uvVal};
  return data;
}

/* 
 * Get UV index with sensor voltage 
 */
uint16_t getUvIndex(uint16_t sensorValue){
  uint16_t index = 0;
  
  // Voltage in miliVolts
  uint16_t voltage = (sensorValue * (5.0 / 1023.0)) * 1000;  
  
  if (voltage >= 0 && voltage < 227) {
    index = 0;
  } else if (voltage >= 227 && voltage < 318) {
    index = 1;
  } else if (voltage >= 318 && voltage < 408) {
    index = 2;
  } else if (voltage >= 408 && voltage < 503) {
    index = 3;
  } else if (voltage >= 503 && voltage < 606) {
    index = 4;
  } else if (voltage >= 606 && voltage < 696) {
    index = 5;
  } else if (voltage >= 696 && voltage < 795) {
    index = 6;
  } else if (voltage >= 795 && voltage < 881) {
    index = 7;
  } else if (voltage >= 881 && voltage < 976) {
    index = 8;
  } else if (voltage >= 976 && voltage < 1079) {
    index = 9;
  } else if (voltage >= 1079 && voltage < 1170) {
    index = 10;
  } else {
    index = 11;
  }
  return index;
}

/*
 * Connect to MTQQ with UID + ID if not already connected
 */
void connect_MQTT() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection with UID: ");
    // Attempt to connect
    char buffer[25]; sprintf(buffer, "%s-%d", MQTT_UID, MQTT_ID);
    Serial.println(buffer);
    
    if (client.connect(buffer)) {
      Serial.println("MQTT connected");          
      // Register MQTT sensors
      if (register_MQTT_sensors) {
        register_MQTT_sensors_to_homeassistant();
        register_MQTT_sensors = false;
      }
    } else {
      // Wait 5 minute before retrying
      MQTT_ID += 1;
      register_MQTT_sensors = true;
      delay(5*60*1000);
    }
  }
}

// Last time bublished to MQTT
long lastMsg = 0;

// To detect if wifi is down
unsigned long previousMillis = 0;
unsigned long interval = 30000;

/* 
 *  Main loop
 */
void loop(void) {
  unsigned long currentMillis = millis();
  
  // if WiFi is down, try reconnecting
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    
    WiFi.disconnect();
    WiFi.reconnect();
    
    // Set up MQTT
    register_MQTT_sensors = true;
    connect_MQTT(); 
     
    previousMillis = currentMillis;
  }
  
  // Handle HTTP requests
  server.handleClient();
  
  // Publish to MTQQ every 1min
  if (currentMillis - lastMsg > 1000*60) {
    lastMsg = currentMillis;
   
    // Read sensors
    DHT22Data temperature = read_dht();
    TSL2561Data luminosity = read_luminosity();
    SGP30Data sgp30 = read_sgp30();
    BME280Data bme280 = read_bme280();

    // Set up MQTT
    connect_MQTT();
    
    if(client.connected()) {    
      // Publish to MQTT
      Serial.println("Staring to publish MTTQ");
      
      JSONVar json = new JSONVar();
      
      // Temperature
      json["temperature"] = temperature.temperature;
      
      // Humidity
      json["humidity"] = temperature.humidity;
      
      // Heat index - Temperature with humidity combined
      json["heatIndex"] = temperature.heatIndex;
      
      // Luminosity
      json["luminosity"] = luminosity.luminosity;
      
      // UV
      json["uv"] = luminosity.uvIndex;

      JSONVar sgp30Json = new JSONVar();
      
      // TVOC
      sgp30Json["tvoc"] = sgp30.tvoc;
      sgp30Json["tvocBase"] = sgp30.tvoc_base;
      
      // CO2
      sgp30Json["co2"] = sgp30.co2;
      sgp30Json["co2Base"] = sgp30.co2_base;

      // H2
      sgp30Json["h2"] = sgp30.h2;
      
      // Ethanol
      sgp30Json["ethanol"] = sgp30.ethanol;

      JSONVar bme280Json = new JSONVar();
      
      // BME280 temperature
      bme280Json["bme280_temp"] = bme280.temperature;

      // BME280 humidity
      bme280Json["bme280_humidity"] = bme280.humidity;

      // BME280 preassure
      bme280Json["bme280_pres"] = bme280.preassure;

      // bme280Json altitude
      bme280Json["bme280_alt"] = bme280.approximateAltitude;
      
     MQTTMessage sensors[] = {
        {json,       "homeassistant/sensor/esp32dev/state"},
        {sgp30Json,  "homeassistant/sensor/esp32dev/SGP30/state"},
        {bme280Json, "homeassistant/sensor/esp32dev/BME/state"}
        
      };
      byte sensorCount = 3;
      
      for (byte i = 0; i < sensorCount; i++) {
        MQTTMessage sensor = sensors[i];
        String jsonString = JSON.stringify(sensor.json);
        byte stringLength = jsonString.length() + 1;
        char payload[stringLength];
        jsonString.toCharArray(payload, stringLength);
        Serial.println(sensor.topic);
        Serial.println(payload);
        client.publish(sensor.topic, payload);
      }

      Serial.println("Finished to publish MTTQ");
    }
  }
};
