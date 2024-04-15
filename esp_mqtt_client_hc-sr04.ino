/*********
  This code is built upon the example code in pubsubclient library 
*********/
// set up HC-SR04 Sensor

//include NewPing Library
#include "NewPing.h"
#define TRIGGER_PIN 5   //UPDATE***********************************
#define ECHO_PIN 18     //UPDATE***********************************

//max distance to ping for (in inches)
#define MAX_DISTANCE 157.58

//NewPing set up of pins and max distance
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

//
float distance, avg_distance, temp_distance, count, height, radius1, volume;
float x = 0;
const float radius2 = 4.625;

// set up MQTT systems
#include <WiFi.h>
#include <PubSubClient.h>

// Replace the SSID/Password details as per your wifi router
const char* ssid = "C-12D";
const char* password = "change4420exact";

// Replace your MQTT Broker IP address here:
const char* mqtt_server = "192.168.0.187";    //UPDATE***********************************

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;

#define ledPin 2

void blink_led(unsigned int times, unsigned int duration){
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(duration);
    digitalWrite(ledPin, LOW); 
    delay(200);
  }
}

void setup_wifi() {
  delay(50);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int c=0;
  while (WiFi.status() != WL_CONNECTED) {
    blink_led(2,200); //blink LED twice (for 200ms ON time) to indicate that wifi not connected
    delay(1000); //
    Serial.print(".");
    c=c+1;
    if(c>10){
        ESP.restart(); //restart ESP after 10 seconds
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
}

void connect_mqttServer() {
  // Loop until we're reconnected
  while (!client.connected()) {

        //first check if connected to wifi
        if(WiFi.status() != WL_CONNECTED){
          //if not connected, then first connect to wifi
          setup_wifi();
        }

        //now attemt to connect to MQTT server
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP32_client1")) { // Change the name of client here if multiple ESP32 are connected
          //attempt successful
          Serial.println("connected");
          // Subscribe to topics here
          client.subscribe("rpi/broadcast");
          //client.subscribe("rpi/xyz"); //subscribe more topics here
          
        } 
        else {
          //attempt not successful
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" trying again in 2 seconds");
    
          blink_led(3,200); //blink LED three times (200ms on duration) to show that MQTT server connection attempt failed
          // Wait 2 seconds before retrying
          delay(2000);
        }
  }
  
}

//this function will be executed whenever there is data available on subscribed topics
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

  // Check if a message is received on the topic "rpi/broadcast"
  if (String(topic) == "rpi/broadcast") {
      if(messageTemp == "10"){
        Serial.println("Action: blink LED");
        blink_led(1,1250); //blink LED once (for 1250ms ON time)
      }
  }

  //Similarly add more if statements to check for other subscribed topics 
}

// method to calculate the volume of a conical tank based on distance of solution in tank to sensor
void volume(){
   int avg_distance = 0;
  count = 0;
  //sets up temp variable to test percent difference
  distance = sonar.ping_in();
  temp_distance = distance;
  
  //initalize an array to store distance for given time frame
  for(int i = 0; i<= 10;i++)
  {
    distance = sonar.ping_in();
    //does not calculate distance in average if +/- 5% of previous distance
    if(distance <= (temp_distance * 1.05) && distance >= (temp_distance * 0.95))
    {
      avg_distance += distance;
      count++;
    } 
  }
  avg_distance = avg_distance / (count) ;
 
   height = 29.75 - avg_distance;  //height equals the total height of the tank minus the distance from the HC-SR04 Sensor    
   radius1 = 0.897435 * height;  //top radius to height ration is: 0.897435
   volume = 3.14 *(sq(radius1) + (radius1 * radius2) + sq(radius2)) * height / 3;
   volume = volume / 61.024; // divide by 61.024 to convert to liters

    return volume;
}

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server,1883);//1883 is the default port for MQTT server
  client.setCallback(callback);
}
/////////


//////////
void loop() {
  
  if (!client.connected()) {
    connect_mqttServer();
  }

  client.loop();
  
  long now = millis();
  if (now - lastMsg > 4000) {
    lastMsg = now;


    client.publish("esp32/sensor1", volume()); //topic name (to which this ESP32 publishes its data). UPDATE*************POTENTIAL ERROR
  }
  
}
