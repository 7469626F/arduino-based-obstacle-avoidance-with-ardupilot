///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
//mavlink libary 
#include "mavlink.h"       
#include "mavlink_msg_distance_sensor.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
//mavlink boudrate
#define bRate 115200


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//sensor setup

/*Copy everything that comes before 'void setup' in your sensor's example sketch over here.*/
/*Do not copy the the code that is used for a serial monitor, only copy the part that is relevant to sensor operation.*/

int distance_1              //the value that will be sent to ardupilot You will need to copy this when a second sensor is attached. Increase the number with one increment every time it is copied.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

void setup()
{
  Serial.begin(bRate);      //start mavlik serial comunication
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //sensor initialization
  
  /*Copy everything that sits within void setup of your sensor's example sketch over here.*/
  /*Do not copy the the code that is used for a serial monitor, only copy the part that is relevant to sensor operation.*/
  
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

void loop() {

  //command_heartbeat();    //debuging
  command_distance_1();     //You will need to copy this when a second sensor is attached. Increase the number with one increment every time it is copied.
  
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
//This is for debugging purposes, no need to touch it unless you really want to.
void command_heartbeat() {

  int sysid = 100;                   
  //< The component sending the message.
  int compid = MAV_COMP_ID_PATHPLANNER;    
  
  uint8_t system_type =MAV_TYPE_GCS;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  
  uint8_t system_mode = 0; 
  uint32_t custom_mode = 0;                
  uint8_t system_state = 0;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(sysid,compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  //delay(1);
  Serial.write(buf, len);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This is the comand that reads the sensor data. You will need to copy this when a second sensor is attached. Increase the number with one increment every time it is copied.
void read_sensor1(){
  /*Copy everything that sits within void loop of your sensor's example sketch over here.*/
  /*Do not copy the the code that is used for a serial monitor, only copy the part that is relevant to sensor operation.*/
  /*make sure that the value you wish to send to ardupilot is in centimeters*/

  distance_1=/*the value you want to sent to ardupilot*/
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This is the mavlink message that is send to ardupilot. You will need to copy this when a second sensor is attached. Increase the number with one increment every time it is copied.
void command_distance_1() {

  read_sensor1();                             //Increase the number with one increment every time it is copied.
  
  int sysid = 1;                              //this is the ardupilot default 

  int compid = 158;    

  uint32_t time_boot_ms = 0;                  //leave this at 0
  uint16_t min_distance = 1;                  //the minimum distance the sensor can measure in centimeters
  uint16_t max_distance = 300;                //the maximum distance the sensor can measure in centimeters
  uint16_t current_distance = distance_1;     //the distance that needs to be sned to ardupilot Increase the number with one increment every time it is copied.
  uint8_t type = 0;                           //leave this at 0
  uint8_t id = 1;                             //the sensor id Increase the number with one increment every time it is copied.
  uint8_t orientation = 0;                    //0=forward, each increment is 45degrees more in clockwise direction, 24: upwards or 25: downwards

  uint8_t covariance = 0;                     //measurement variance. max standard deviation is 6cm. 255 if unknown.


  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); 

  Serial.write(buf, len);
}
