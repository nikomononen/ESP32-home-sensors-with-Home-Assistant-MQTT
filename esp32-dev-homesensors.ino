#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include "SGP30.h"
#include <DHT.h>
#include <Arduino_JSON.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include "esp32-hal-adc.h" // needed for adc pin reset

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

struct SPG30Data {
  float tvoc;
  uint16_t tvoc_base;
  float co2;
  uint16_t co2_base;
  float h2;
  float ethanol;
};

// DHT 22 temperature and humidity sensor
#define DHTTYPE DHT22   // DHT 11 sensor 
int DHTPIN = 4;

// UV sensor
int UVPIN = 33;

// MQTT Broker IP address
IPAddress mqtt_server(0, 0, 0, 0);
const char* MQTT_UID = "<MQTT-UID>";

// Wifi
const char* ssid = "<ssid>";
const char* password = "<password>";

WebServer server(80);
// MQTT client attached to wificlient
WiFiClient espClient;

/*
 * MQTT callback handler
 */
void callback(char* topic, byte* message, unsigned int length) {
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

PubSubClient client(mqtt_server, 1883, callback, espClient);

// SGP30 and DHT22 sensors
SGP30 SGP;
DHT dht(DHTPIN, DHTTYPE);

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
// TSL2561 Luminosity Sensor
// I2C address:
#define TSL2561_SEL_LOW 0x29
#define TSL2561_ADDR_FLOAT 0x39
#define TSL2561_HIGH 0x49
#define TSL2561_UNIQUE_ID 12345
// The address will be different depending on whether you leave
// the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, TSL2561_UNIQUE_ID);

// Device setup, run once
void setup(void) {
  Serial.begin(115200);
  
  // Start DHT22 sensor
  dht.begin();
  
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

  // Set up MTQQ
  if(client.connect(MQTT_UID)) {
    register_MQTT_sensors_to_homeassistant();
  }
  
  // Start HTTP server and set handlers
  server.on("/", handleRoot);
  
  // Temperature
  server.on("/DHT22", handleDHT22);
  
  // Luminousity
  server.on("/luminosity", handleTSL2561);

  server.onNotFound(handleNotFound);
  server.begin();
  
  Serial.println("HTTP server started"); 
  
  // Configure TSL2561 Luminousity sensor
  if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  configure_tls25621_sensor();
  
  // Configure SGP32 sensor
  configure_sgp30_sensor();
}

void register_MQTT_sensors_to_homeassistant(){
  Serial.println("MTQQ connected. Registering Home assistant sensors");
  // Create Home Assistant sensors
  JSONVar json;
  
  // Temperature
  json["unique_id"] = "esp32-dev-temperature";
  //json["device_class"] = "temperature";
  json["name"] = "Humidity";
  json["state_topic"] = "homeassistant/sensor/esp32dev/state";
  json["unit_of_measurement"] = "%";
  json["value_template"] = "{{ value_json.humidity}}";
  String jsonString = JSON.stringify(json);
  char plain[jsonString.length()+1];
  jsonString.toCharArray(plain, jsonString.length()+1);
  client.publish("homeassistant/sensor/esp32dev/Temperature/config", plain);
  
  // Humidity
  json = new JSONVar();
  json["unique_id"] = "esp32-dev-humidity";
  //json["device_class"] = "humidity";
  json["name"] = "Temperature";
  json["state_topic"] = "homeassistant/sensor/esp32dev/state";
  json["unit_of_measurement"] = "%";
  json["value_template"] = "{{ value_json.temperature}}";
  jsonString = JSON.stringify(json);
  plain[jsonString.length()+1];
  jsonString.toCharArray(plain, jsonString.length()+1);
  Serial.println(plain);
  client.publish("homeassistant/sensor/esp32dev/Humidity/config", plain);
  
  // Feels like
  json = new JSONVar();
  json["unique_id"] = "esp32-dev-feelslike";
  //json["device_class"] = "temperature";
  json["name"] = "Feels like";
  json["state_topic"] = "homeassistant/sensor/esp32dev/state";
  json["unit_of_measurement"] = "°C";
  json["value_template"] = "{{ value_json.heatIndex}}";  
  jsonString = JSON.stringify(json);
  plain[jsonString.length()+1];
  jsonString.toCharArray(plain, jsonString.length()+1);
  Serial.println(plain);
  client.publish("homeassistant/sensor/esp32dev/Heatindex/config", plain);   
  
  // Luminosity
  json = new JSONVar();
  json["unique_id"] = "esp32-dev-illuminance";
  //json["device_class"] = "illuminance";
  json["name"] = "Luminosity";
  json["state_topic"] = "homeassistant/sensor/esp32dev/state";
  json["unit_of_measurement"] = "lux";
  json["value_template"] = "{{ value_json.luminosity}}";
  jsonString = JSON.stringify(json);
  plain[jsonString.length()+1];
  jsonString.toCharArray(plain, jsonString.length()+1);
  Serial.println(plain);
  client.publish("homeassistant/sensor/esp32dev/Luminosity/config", plain);
  
  // UV light
  json = new JSONVar();
  json["unique_id"] = "esp32-dev-uvIndex";
  json["name"] = "UV light";
  json["state_topic"] = "homeassistant/sensor/esp32dev/state";
  json["unit_of_measurement"] = "";
  json["value_template"] = "{{ value_json.uv}}";  
  jsonString = JSON.stringify(json);
  plain[jsonString.length()+1];
  jsonString.toCharArray(plain, jsonString.length()+1);
  Serial.println(plain);
  client.publish("homeassistant/sensor/esp32devUvIndex/config", plain);
  Serial.println("Home assistant sensors registered.");

  // TVOC light
  json = new JSONVar();
  json["unique_id"] = "esp32-dev-tvoc";
  json["name"] = "TVOC";
  json["state_topic"] = "homeassistant/sensor/esp32dev/state";
  json["unit_of_measurement"] = "";
  json["value_template"] = "{{ value_json.tvoc}}";  
  jsonString = JSON.stringify(json);
  plain[jsonString.length()+1];
  jsonString.toCharArray(plain, jsonString.length()+1);
  Serial.println(plain);
  client.publish("homeassistant/sensor/esp32devTVOC/config", plain);

  // CO2
  json = new JSONVar();
  json["unique_id"] = "esp32-dev-co2";
  json["name"] = "CO2";
  json["state_topic"] = "homeassistant/sensor/esp32dev/state";
  json["unit_of_measurement"] = "";
  json["value_template"] = "{{ value_json.co2}}";  
  jsonString = JSON.stringify(json);
  plain[jsonString.length()+1];
  jsonString.toCharArray(plain, jsonString.length()+1);
  Serial.println(plain);
  client.publish("homeassistant/sensor/esp32devCO2/config", plain);
  
  // H2
  json = new JSONVar();
  json["unique_id"] = "esp32-dev-h2";
  json["name"] = "H2";
  json["state_topic"] = "homeassistant/sensor/esp32dev/state";
  json["unit_of_measurement"] = "";
  json["value_template"] = "{{ value_json.h2}}";  
  jsonString = JSON.stringify(json);
  plain[jsonString.length()+1];
  jsonString.toCharArray(plain, jsonString.length()+1);
  Serial.println(plain);
  client.publish("homeassistant/sensor/esp32devH2/config", plain);
  
  // Ethanol
  json = new JSONVar();
  json["unique_id"] = "esp32-dev-ethanol";
  json["name"] = "Ethanol";
  json["state_topic"] = "homeassistant/sensor/esp32dev/state";
  json["unit_of_measurement"] = "";
  json["value_template"] = "{{ value_json.ethanol}}";  
  jsonString = JSON.stringify(json);
  plain[jsonString.length()+1];
  jsonString.toCharArray(plain, jsonString.length()+1);
  Serial.println(plain);
  client.publish("homeassistant/sensor/esp32devEthanol/config", plain);
  
  Serial.println("Home assistant sensors registered.");
}

// Configures the gain and integration time for the TSL2561
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

void configure_sgp30_sensor(){
  Serial.println(SGP30_LIB_VERSION);
  Serial.println();

  Serial.print("BEGIN:\t");
  Serial.println(SGP.begin());
  Serial.print("TEST:\t");
  Serial.println(SGP.measureTest());
  Serial.print("FSET:\t");
  Serial.println(SGP.getFeatureSet(), HEX);
  SGP.GenericReset();

  Serial.print("DEVID:\t");
  SGP.getID();
  
  for (int i = 0; i < 6; i++){
    if (SGP._id[i] < 0x10) Serial.print(0);
    Serial.print(SGP._id[i], HEX);
  }
  
  uint16_t bl_co2 = 0;
  uint16_t bl_tvoc = 0;
  bool b = SGP.getBaseline(&bl_co2, &bl_tvoc);
    
  Serial.println();
}

/*
 * HTTP server handlers
 */
void handleRoot() {
  server.send(200, "text/plain", "hello from esp8266!");
}
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

SPG30Data read_spg30() {
  // First 10-20 readings will always be TVOC 0 ppb eCO2 400 ppm. 
  SGP.measure(true);  
  
  uint16_t tvoc_base =0;
  uint16_t co2_base = 0;
  SGP.getBaseline(&co2_base, &tvoc_base);
  
  float tvoc = SGP.getTVOC();
  float co2 = SGP.getCO2();
  float h2 = SGP.getH2();
  float ethanol = SGP.getEthanol();
  
  SPG30Data data = {tvoc, tvoc_base, co2, co2_base, h2, ethanol};
  return data;
}

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
  int uvVal = digitalRead(UVPIN);
  
  TSL2561Data data = {event.light, broadband, infrared, uvVal};
  return data;
}

/*
 * Helper to transform String to charArray
 */
const char* stringToCharArray(String message){
  char plain[message.length()];
  message.toCharArray(plain, message.length());
  return plain;
}

// Last time bublished to MQTT
long lastMsg = 0;

// To detect if wifi is down
unsigned long previousMillis = 0;
unsigned long interval = 30000;

// Main loop
void loop(void) {
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    
    WiFi.disconnect();
    WiFi.reconnect();
    
    if( client.connect(MQTT_UID)) {
      Serial.println("MTQQ connected");
    }
    
    previousMillis = currentMillis;
  }
  
  // Handle HTTP requests
  server.handleClient();
  
  // Publish to MTQQ every 1min
  if (currentMillis - lastMsg > 1000*60) {
    lastMsg = currentMillis;
    Serial.println("Staring to publish MTTQ");
    
    // Read sensors
    DHT22Data temperature = read_dht();
    TSL2561Data luminosity = read_luminosity();
    SPG30Data spg30 = read_spg30();
    
    // Set up MTQQ
    if( client.connect(MQTT_UID)) {
      Serial.println("MTQQ connected");
      
      // Register sensors. This is needed if homeassistant is restarted so that sensor data shows up again
      register_MQTT_sensors_to_homeassistant();
      
      // Publish to MQTT
      JSONVar json;
      
      // Temperature
      json["temperature"] = temperature.temperature;
      
      // Humidity
      json["humidity"] = temperature.humidity;
      
      // Heat index - Temperature with humidity combined
      json["heatIndex"] = round(temperature.heatIndex);
      
      // Luminosity
      json["luminosity"] = luminosity.luminosity;
      
      // UV
      json["uv"] = luminosity.uvIndex;

      // TVOC
      json["tvoc"] = spg30.tvoc;
      json["tvocBase"] = spg30.tvoc_base;
      
      // CO2
      json["co2"] = spg30.co2;
      json["co2Base"] = spg30.co2_base;

      // H2
      json["h2"] = spg30.h2;
      
      // Ethanol
      json["ethanol"] = spg30.ethanol;
      
      // Return JSON
      String jsonString = JSON.stringify(json);
      char plain[jsonString.length()+1];
      jsonString.toCharArray(plain, jsonString.length()+1);
      Serial.println(plain);
      client.publish("homeassistant/sensor/esp32dev/state", plain);
      Serial.println("Finished to publish MTTQ");
    }
  }
};
