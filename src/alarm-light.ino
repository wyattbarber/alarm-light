#include <Arduino.h>
#include <ESP8266WiFi.h>

#include "AlarmLight.h"
#include <Time.h>

#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <smarthome/Register.h>
#include <smarthome/Execute.h>
#include <smarthome/Device.h>
#include <smarthome/Query.h>

#define TWELVE_V_PIN D3
#define RED_PIN D4
#define GRN_PIN D5
#define BLU_PIN D6
#define WHT_PIN D7
#define BUZ_PIN D8
#define MAT_PIN A0

// ROS data
String dev_id = "wakelight";
ros::NodeHandle nh;
std_msgs::Int32 tMsg;
ros::Publisher test("test_topic", &tMsg);


// Wifi access data
const char *ssid = "the wifi";
const char *pswd = "the wifi password";
IPAddress server(192, 168, 0, 105);
uint16_t port = 11411;


// Alarm, max illumination
SmartHome::AlarmLight *alarm;    
int maxLights[] = {1023, 1023, 1023, 1023};

/* Handler for EXECUTE requests from server
 * 
 * Turns the alarm on or off based on request content
 */
bool execute_handler(smarthome::Execute::Request& req, smarthome::Execute::Response& res){
  if(req.command == "action.devices.commands.OnOff") {
    if(req.param_values[0] == "true") {
       nh.loginfo(dev_id + " --- Turning light ON.");
       alarm->on();
    }
    else {
       nh.loginfo(dev_id + "--- Turning light OFF.");
       alarm->off();
    }
  }
  else {
    nh.logerror(dev_id + " --- Unregognized command: " + req.command);
    return false;
  }
  return true;
}
ros::ServiceServer<smarthome::Execute::Request, smarthome::Execute::Response> execute_srv(dev_id+"/execute", &execute_handler);

/* Handler for QUERY requests from server
 *  
 * Responds with simple on/off status, rather than buzzer or brightness
 */
bool query_handler(smarthome::Query::Request& req, smarthome::Query::Response& res){
  nh.loginfo(dev_id + " --- QUERY request recieved.");
  res.param_names = { "on" };
  res.param_values = { (on ? "true" : "false") };
  return true;
}
ros::ServiceServer<smarthome::Query::Request, smarthome::Query::Response> query_srv(dev_id+"/query", &query_handler);

/* Client for REGISTER service
 * 
 * Registers this device with the server
 */
 ros::ServiceClient<smarthome::Register::Request, smarthome::Register::Response> register_srv("register_device");

void setup()
{
    // Start serial for debugging
    Serial.begin(9600);

    // Construct alarm
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
    WiFi.begin(ssid, pswd);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi connecting... ");
        delay(1000);
    }

    // Setup ROS
    nh.getHardware()->setConnection(server, port);
    nh.initNode();
    nh.serviceServer(execute_srv);
    nh.serviceServer(query_srv);
    nh.serviceClient(register_srv);
    // Wait for ROS to connect
    while(!nh.connected()){
      nh.spinOnce();
    }
    // Get parameters

    // Register with server
    smarthome::Register::Request reg_req;
    smarthome::Register::Response reg_res;
    
    reg_req.device.id =                 dev_id;
    reg_req.device.type =               "action.devices.types.LIGHT";
    reg_req.device.traits =             {"action.devices.traits.OnOff"};
    reg_req.device.name =               dev_id;
    reg_req.device.nicknames =          {"wake light", "fade light"};
    reg_req.device.default_names =      {"alarm light"};
    reg_req.device.will_report_state =  false;
    reg_req.device.room_hint =          "Bedroom";
    reg_req.device.manufacturer =       "Wyatt Corp.";
    reg_req.device.model =              "Development";
    reg_req.device.sw_version =         "0.0.1";
    reg_req.device.hw_version =         "0.0.0";
    reg_req.device.device_id =          dev_id;

    while(true) {
      if(register_srv.call(reg_req, reg_res)){
        Serial.println("Registration with server succeeded");
        break;
      }
      else {
        Serial.println("Registration with server failed. Retry in 1 second.");
        ros::Duration(1.0).sleep();
      }
    }
}

void loop()
{
    // Update alarm status
    alarm->update();
    int* rgbw = alarm->getLights();

    // Set 12V relay output
    if(
        (rgbw[0] != 0)
    ||  (rgbw[1] != 0)
    ||  (rgbw[2] != 0)
    ||  (rgbw[3] != 0)
    )
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

    // Log status for debugging
    Serial.print("Lights: ");
    Serial.print(rgbw[0]);
    Serial.print(", ");
    Serial.print(rgbw[1]);
    Serial.print(", ");
    Serial.print(rgbw[2]);
    Serial.print(", ");
    Serial.print(rgbw[3]);
    Serial.print(". \n");

    nh.spinOnce();
}

time_t getTime()
{
    return nh.now().sec;
}
