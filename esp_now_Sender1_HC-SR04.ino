/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

/********HC-SR04********/
//include NewPing Library
#include "NewPing.h"
#define TRIGGER_PIN 5
#define ECHO_PIN 18

//max distance to ping for (in inches)
#define MAX_DISTANCE 157.58

//NewPing set up of pins and max distance
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
float distance, avg_distance, temp_distance, count, height, radius1, volume;
float x = 0;
const float radius2 = 4.625;


//calculate average distance
float calculate_volume()
{
  int avg_distance = 0;
  count = 0;
  //sets up temp variable to test percent difference
  distance = sonar.ping_in();
  temp_distance = distance;
  
  //initalize an array to store distance for given time frame
  for(int i = 0; i<= 10;i++){
    distance = sonar.ping_in();
    //does not calculate distance in average if +/- 5% of previous distance
    if(distance <= (temp_distance * 1.05) && distance >= (temp_distance * 0.95)){
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

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x28, 0xCF, 0xC4};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    float volume;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Set values to send
  myData.id = 1;
  float v = calculate_volume();
  myData.volume = v;
  
  Serial.println(myData.volume);

  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(10000);
}
