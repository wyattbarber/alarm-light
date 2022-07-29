/*
    "Real" ros node for the alarm-light package. 
    Handles service implementations required by the smarthome package.
    Interfaces with a esp8266 module via std_msgs topic types.
    Requires esp module to be already connected via rosserial.
*/  
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <smarthome/Register.h>
#include <smarthome/Query.h>
#include <smarthome/Execute.h>

// Device info
bool status;
std::string dev_id = "wakelight";
std_msgs::Bool cmd;

// ROS data
ros::Publisher command;


/* Handler for EXECUTE requests from server
 * 
 * Turns the alarm on or off based on request content
 */
bool execute_handler(smarthome::Execute::Request& req, smarthome::Execute::Response& res){
  if(req.command == "action.devices.commands.OnOff") {
    // Check command
    if(req.param_values[0] == "true") {
       ROS_INFO_STREAM(dev_id << " --- Turning light ON.");
       status = true;
    }
    else {
       ROS_INFO_STREAM(dev_id << "--- Turning light OFF.");
       status = false;
    }
    // Send command
    cmd.data = status;
    command.publish(cmd);
  }
  else {
    ROS_ERROR_STREAM(dev_id << " --- Unregognized command: " << req.command);
    return false;
  }
  return true;
}

/* Handler for QUERY requests from server
 *  
 * Responds with simple on/off status, rather than buzzer or brightness
 */
bool query_handler(smarthome::Query::Request& req, smarthome::Query::Response& res){
  ROS_INFO_STREAM(dev_id << " --- QUERY request recieved.");
  res.param_names = { "on" };
  res.param_values = { (status ? "true" : "false") };
  return true;
};


int main(int argc, char* argv[]){
    // ROS setup
    ros::init(argc, argv, dev_id);
    ros::NodeHandle nh;
    ros::ServiceServer query_srv = nh.advertiseService(dev_id+"/query", &query_handler);
    ros::ServiceClient register_srv = nh.serviceClient<smarthome::Register::Request, smarthome::Register::Response>("register_device");
    ros::ServiceServer execute_srv = nh.advertiseService(dev_id+"/execute", &execute_handler);
    command = nh.advertise<std_msgs::Bool>(dev_id+"/arduino/cmd", 1);

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
        ROS_INFO("Node alarm-light registered with smarthome server.");
        break;
      }
      else {
        ros::Duration(1.0).sleep();
        ROS_ERROR("Registration with smarthome server failed. Retry in 1 second.");
      }
    }

    ros::spin();

    return 0;
}