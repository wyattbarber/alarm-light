
#include <Arduino.h>
#include <ESP8266WiFi.h>

#include "AlarmLight.h"
#include <Time.h>

#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>

#define TWELVE_V_PIN D3
#define RED_PIN D4
#define GRN_PIN D5
#define BLU_PIN D6
#define WHT_PIN D7
#define BUZ_PIN D8


// Alarm, max illumination
SmartHome::AlarmLight *alarm;    
int maxLights[] = {1023, 1023, 1023, 1023};

// ROS data
ros::NodeHandle nh;
// Command subscriber
std_msgs::Bool cmd;
void executeCallback(const std_msgs::Bool& msg){
    if(msg.data) {
       nh.loginfo("wakelight:esp --- Turning light ON.");
       alarm->On();
    }
    else {
       nh.loginfo("wakelight:esp --- Turning light OFF.");
       alarm->Off();
    }
}
ros::Subscriber<std_msgs::Bool> execute("wakelight/arduino/cmd", &executeCallback);
// Heartbeat publisher
std_msgs::Bool hb;
ros::Publisher heartbeat("wakelight/arduino/heartbeat", &hb);

// Wifi access data
const char *ssid = "the wifi";
const char *pswd = "the wifi password";
IPAddress server(192, 168, 0, 110);
uint16_t port = 11411;

void setup()
{
    // Start serial for debugging
    Serial.begin(9600);
    Serial.println("Beginning...");

    // Construct alarm
    Serial.println("Making alarm...");
    alarm = new SmartHome::AlarmLight(maxLights);
    setSyncProvider(getTime); // timeClient.update() called in getNtpTime()
    setSyncInterval((time_t)60); // getNtpTime() called every minute
    
    // Set pin modes
    pinMode(TWELVE_V_PIN, OUTPUT);
    pinMode(RED_PIN, OUTPUT);
    pinMode(BLU_PIN, OUTPUT);
    pinMode(GRN_PIN, OUTPUT);
    pinMode(WHT_PIN, OUTPUT);

    // Begin WiFi
    Serial.println("Adding WiFi...");
    WiFi.begin(ssid, pswd);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi connecting... ");
        delay(1000);
    }

    // Setup ROS
    nh.getHardware()->setConnection(server, port);
    nh.initNode();
    nh.subscribe(execute);
    nh.advertise(heartbeat);
    
    // Wait for ROS to connect
    while(!nh.connected()){
      Serial.println("ROS connecting... ");
      nh.spinOnce();
    }
    // Get parameters 
}

void loop()
{
    Serial.println("Looping...");
    
    // Update alarm status
    alarm->update();
    int* rgbw = alarm->getLights();

    // Set 12V relay output
    if(alarm->isOn())
    {
        digitalWrite(TWELVE_V_PIN, HIGH);
    }
    else
    {
        digitalWrite(TWELVE_V_PIN, LOW);
    }
    
    // Set light outputs
    analogWrite(RED_PIN, rgbw[0]);
    analogWrite(BLU_PIN, rgbw[1]);
    analogWrite(GRN_PIN, rgbw[2]);
    analogWrite(WHT_PIN, rgbw[3]);

    // Set buzzer output
    digitalWrite(BUZ_PIN, alarm->getBuzzer());

    nh.spinOnce();
}

time_t getTime()
{
    return nh.now().sec;
}
