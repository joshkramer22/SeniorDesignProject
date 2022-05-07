#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
//#include <Arduino_JSON.h>

//#include <SAMD_AnalogCorrection.h>

#include "DHT.h"   // This includes the DHT Libaries from Adafruit to use the DHT11
#define DHTPIN 7    // This is the DHT-11 Output Pin Connection
#define DHTTYPE DHT11   // This is the DHT-11 Type
DHT dht11(DHTPIN, DHTTYPE);   // Sets up the DHT sensor

/*
 Data Type    | Variable       | Range      | Unit
 -------------------------------------------------------
 float        | humidity    | (0-100)       | Celsius
 float        | temp        | (0-100)       | Percentage
 unsigned int | num_minutes | less than 60  | Minutes
  
*/


char ssid[] = "";        // your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "";
int        port     = 1883;
const char topic[]  = "myArduinoTopic/in0";
const char topic1[]  = "myArduinoTopic/in1";
const char topic2[]  = "myArduinoTopic/in2";
const char topic3[]  = "myArduinoTopic/in3";

const long interval = 1800000;
unsigned long previousMillis = 0;

int count = 0;


int phPinP = A1;
int phPinN = A2;
int buf[10], tempPH;
unsigned long int avgValue;
float avgPHval = 0;
int phValCount = 0;

  // Define Variables
  float humidity;    // Stores humidity values in Percent format
  float temp;    // Stores temperature values in Celsius format
  unsigned long previous_millis = 0;    // Stores previous time
  unsigned int num_minutes = 1;    // (Editable) Stores number of minutes, set to 15 minutes

void setup() 
{
  Serial.begin(9600);

    // Initialize the DHT-11
  dht11.begin();
  
  analogReadResolution(12); //read in 12 bits of analog values
//  analogReadCorrection(7, 2058);

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
  
}

/*
TRENDLINE FOR PH MEASURED VOLTAGES:
    y = -0.0005x^2 - 0.0136x + 0.1148

AIR ANALOG VALUE : 1.36 - 1.39
 0.1148 - 1.37 = -1.2552

NEW TRENDLINE:
    y = -0.0005x^2 - 0.0136x -1.2552

*/


void loop() 
{
  
  float phValueP = 1.0, phValueN = 1.0;
  float adjustedValP = 1.0, adjustedValN = 1.0;
  float phVal = 7.0;
 
    phValueP = analogRead(phPinP);
    adjustedValP = phValueP * (5.0/4095.0); //12 bit analog input so value between 0 and 4095 and can read up to 5V (10 bit b/t 0 & 1023)

  Serial.print("Non adjusted Pos phValue: ");
  Serial.print(phValueP);
  Serial.print("\t\t Adjusted Pos pH Value: ");
  Serial.println(adjustedValP);

  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(phPinP);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        tempPH=buf[i];
        buf[i]=buf[j];
        buf[j]=tempPH;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
  {
    avgValue+=buf[i];
  }
  float phValue=(float)avgValue * 5.0 / 4095.0 / 6.0; //convert the analog into millivolt
  
  Serial.print("    Voltage:");  
  Serial.print(phValue,6);

  
  //double phPrediction = (-55.361 * pow(phValue,2)) + (129.88 * phValue) - 57.735; //for the 220KOhm Resistors 3.3V
  //double phPrediction = (-55.361 * pow(phValue,2)) + (122.13 * phValue) - 48.914; //for the 1.2MOhm Resistors 3.3V
  //double phPrediction = (-55.361 * pow(phValue,2)) + (232.85 * phValue) - 226.4;  //for 220KOhm Resistors 5.0V
  //double phPrediction = (-55.361 * pow(phValue,2)) + (210.71 * phValue) - 182.05; //for the 1.2MOhm WL RR Resistors 5.0V
  double phPrediction = (-55.361 * pow(phValue,2)) + (243.92 * phValue) - 250.24; //for the 1.2MOhm RL WR Resistors 5.0V
  avgPHval = avgPHval + phPrediction;
  phValCount++;
  Serial.print("                pH Value: ");

  Serial.println(phPrediction, 6);
  float vDiff = phValue - 2.66;
  Serial.print("                V Diff: ");
  Serial.println(vDiff, 4);

// __________________________________________________________________________________________________________________________________\\

  // Stores current time
  unsigned long current_millis = millis();
  
  // Code accounting for 49 day rollover, set to check sensors every 1 minute
  if((unsigned long)(current_millis - previous_millis) >= num_minutes*10000)
  {
    // Get Humidity value
    humidity = dht11.readHumidity();
    //humidity = 30.23;
    
    // Get Temperature value
    temp = dht11.readTemperature();
    //temp = 83.2;
    
    // Print the data on the Serial Monitor
      // Print Humidity values
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
      // Print Temperature values
      Serial.print("Temperature: ");
      Serial.print(temp);
      Serial.println(" C");
      // Newline for formatting
      Serial.println(" ");
  }
  Serial.println();
  Serial.println();


  
    // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();

  // avoid having delays in loop, we'll use the strategy from BlinkWithoutDelay
  // see: File -> Examples -> 02.Digital -> BlinkWithoutDelay for more info
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) 
  {
    avgPHval = avgPHval / phValCount;
    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    Serial.print("PH TOTAL PREDICTION: ");
    Serial.println(phPrediction);
    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
    Serial.println();
    Serial.println();
    phValCount = 0;
  }
  
  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    Serial.print("Sending message to topic: ");
    Serial.println(topic);
    Serial.print("hello ");
    Serial.println(count);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    mqttClient.print("{\"BCG\":\"Entry");
    mqttClient.print(count);
    mqttClient.print("\"");
    mqttClient.endMessage();

    delay(1500);

    mqttClient.beginMessage(topic1);
    mqttClient.print("\"temp\":");
    mqttClient.print(temp);
    mqttClient.endMessage();

    delay(1500);
    
    mqttClient.beginMessage(topic2);
    mqttClient.print("\"humidity\":");
    mqttClient.print(humidity);
    mqttClient.endMessage();

    delay(1500);

    mqttClient.beginMessage(topic3);
    mqttClient.print("\"ph\":");
    mqttClient.print(avgPHval);
    mqttClient.endMessage();
    avgPHval = 0.0;
    
    Serial.println();

    count++;
  }
  
  delay(1500);
}
