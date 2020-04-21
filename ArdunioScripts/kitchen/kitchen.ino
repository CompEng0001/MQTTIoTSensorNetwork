#include <PubSubClient.h> // Connect and publish to the MQTT broker
#include <WiFiNINA.h>        // enable WiFi for MKR1010
#include <math.h>            // for some math functions
#include <seeed_bme680.h>    // for BME680 sensor
#include <ArduinoLowPower.h>   // enable powersaving 

/* WiFi and Thingspeak variables */
char ssid[] = "BT-QXA277"; // replace with your wifi ssid
char pass[] = "G67pDKimraqUpQ"; // replace with your ssid passwor

// MQTT
const char* mqtt_server = "192.168.1.171";  // IP of the MQTT broker
const char* mqtt_username = "mayfield"; // MQTT username
const char* mqtt_password = "IoTNetwork"; // MQTT password

const char* clientID = "kitchen/"; // MQTT client ID change to your own client and ammend each topic to match 
const char* humidity_topic = "kitchen/humidity";
const char* temperature_topic = "kitchen/temperature";
const char* pressure_topic = "kitchen/pressure";
const char* iaq_topic = "kitchen/iaq";
const char* mq5_topic = "kitchen/mq5";
const char* dust_topic = "kitchen/dust";

boolean sendData = false;

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1883, wifiClient); 

/* BME680 Sensor variables */
float temp, hum, bar, voc, IAQ_Value;
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define IIC_ADDR uint8_t(0x76)
Seeed_BME680 bme680(IIC_ADDR); //IIC address remember 0x76

/* Dust Sensor variables */
int pin = 5; // Digital Pin 0 of Ardunio
unsigned long duration;
unsigned long starttime;
float concentration = 0;

/* MQ5 Sensor variables */
#define GAS_SENSOR A0 // Analogue pin 0 of Arduino
float gasValue = 0.0;

void setup() {
  Serial.begin(9600);

  pinMode(pin, INPUT); // Set Dust Sensor pin to INPUT

  // Initialized false by default
  while (!bme680.init())
  {
    Serial.println("bme680 init failed ! can't find device!");
    delay(10000);
  }

  Serial.println("");  Serial.println("BMP280 is connected");

  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, pass);

  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Debugging - Output the IP Address of the ESP8266
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  connect_MQTT();
  
}

void loop() 
{
  if(!client.loop())
  {
    client.disconnect();
    
    connect_MQTT();
  }

    client.loop();
  //connect_MQTT();
    if(sendData == true)      
    {
      getBMEValues();
      getGasData();
      getDustConcentration();
      debugData();
      MQTTSendData();
      sendData = false;
    }
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  if(String(topic).equals(clientID))
  {
    sendData = true;
  } 
}

// Custom function to connet to the MQTT broker via WiFi
void connect_MQTT()
{
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  if (client.connect(clientID, mqtt_username, mqtt_password))
  {
    Serial.println("Connected to MQTT Broker!");
    client.subscribe(clientID);
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }

}

void MQTTSendData()
{
  // MQTT can only transmit strings
  String hs="Hum: "+String((float)hum)+" % ";
  String ts="Temp: "+String((float)temp)+" C ";
  String bs="Pressure: "+String((float)bar)+" hPa ";
  String is="IAQ: "+String((float)IAQ_Value);

  // PUBLISH to the MQTT Broker (topic = Temperature, defined at the beginning)
  if (client.publish(temperature_topic, String(temp).c_str())) {
    Serial.println("Temperature sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("Temperature failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(temperature_topic, String(temp).c_str());
  }

  // PUBLISH to the MQTT Broker (topic = Humidity, defined at the beginning)
  if (client.publish(humidity_topic, String(hum).c_str())) {
    Serial.println("Humidity sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("Humidity failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(humidity_topic, String(hum).c_str());
  }

  // PUBLISH to the MQTT Broker (topic = Pressure, defined at the beginning)
  if (client.publish(pressure_topic, String(bar).c_str())) {
    Serial.println("Pressure sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("Pressure failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(pressure_topic, String(bar).c_str());
  }

  // PUBLISH to the MQTT Broker (topic = IAQ, defined at the beginning)
  if (client.publish(iaq_topic, String(IAQ_Value).c_str())) {
    Serial.println("IAQ sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("IAQ failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(iaq_topic, String(IAQ_Value).c_str());
  }

  // PUBLISH to the MQTT Broker (topic = IAQ, defined at the beginning)
  if (client.publish(mq5_topic, String(gasValue).c_str())) {
     Serial.println("MQ5 sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("MQ5 failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(mq5_topic, String(gasValue).c_str());
  }

  // PUBLISH to the MQTT Broker (topic = IAQ, defined at the beginning)
  if (client.publish(dust_topic, String(concentration).c_str())) {
     Serial.println("Dust sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("Dust failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(dust_topic, String(concentration).c_str());
  }
  //delay(1000*60);       // print new values every 1 Minute
}

/**
  getBMEValues gets the temp, humidity, pressure and VOC
  VOC is Calculated to be returned as IAQ_Value
*/
void getBMEValues()
{
  if (bme680.read_sensor_data()) 
  {
    //debugging only
    Serial.println("Failed to perform reading :(");
    return;
  }
  temp = bme680.sensor_result_value.temperature;         // Get the Temperature reading from the BMP280
  bar = bme680.sensor_result_value.pressure / 100.0;   // Get the Pressure reading from the BMP280

  hum = bme680.sensor_result_value.humidity;
  voc += bme680.sensor_result_value.gas;

  // Calculate the IAQ Value
  int hum_ref = 40, gas_ref = 0, gasLL = 5000, gasUL = 50000;
  float humscore=0.00, gasscore=0.00;

  // Get humidity score between 0.00 and 100.00 
  if(hum >= 38 && hum <=42) { humscore = 0.25 *100; }
  else if (hum < 38) { humscore = ((0.25/hum_ref)*hum)*100; }
  else { humscore = (((-0.25/(100-hum_ref))*hum)+0.416666)*100; }

  // Get VOC score between 0.00 and 100.00
  if(voc >= gasUL) { gas_ref = gasUL; } 
  else if (voc <= gasLL) { gas_ref = gasLL; }
  else { gas_ref = (int)voc;}
  gasscore = (0.75/(gasUL-gasLL)*gas_ref - (gasLL*(0.75/(gasUL-gasLL))))*100;
  
  // Get IAQ Value between 0.00 and +300.00%
  IAQ_Value = (100-(humscore+gasscore))*5;
}

/**
  getGasData acquires the gas ratio which indicates what gas is being detected
  TODO - find the relationship between RS/RO for each gas in PPM?
*/
void getGasData()
{
  float sensor_volt;
  float RS_gas; // Get value of RS in a GAS
  float ratio;  // Get ratio RS_GAS/RS_air
  int sensorValue = analogRead(GAS_SENSOR);
  sensor_volt = (float)sensorValue / 1024 * 5.0; /// here we want to get the voltage at the out. 
  RS_gas = (5.0 - sensor_volt) / sensor_volt; // omit *RL
  float RO = RS_gas / 6.5;
  /*-Replace the name "R0" with the value of R0 in the demo of First Test -*/
  gasValue = RS_gas / RO; // ratio = RS/R0
}

/**
 * getDustConcentration() retrieves the the approx matter in the air as it passes through the laser.
 * The takes 30 seconds, no way around it. 
*/
void getDustConcentration()
{
  bool data = false;
  unsigned long lowpulseoccupancy = 0;
  unsigned long sampletime_ms = 30000; //sampe 30s
  
  while (!data)
  {
    duration = pulseIn(pin, LOW);
    lowpulseoccupancy = lowpulseoccupancy + duration;
    if ((millis() - starttime) > sampletime_ms) //if elapsed time is > sampel time == 30s
    {
      float ratio = lowpulseoccupancy / (sampletime_ms * 10.0);                             // Integer percentage 0=>100
      concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
      lowpulseoccupancy = 0;
      starttime = millis();
      data = true;
    }
  }
}

void debugData()
{
  Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.println(" %");
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" *C");
  Serial.print("Pressure: ");
  Serial.print(bar);
  Serial.println(" hPa");
  Serial.print("IAQ Vaule: ");
  Serial.println(IAQ_Value);
}
