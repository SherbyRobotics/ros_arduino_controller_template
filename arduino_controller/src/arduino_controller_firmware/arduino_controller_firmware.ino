//=========================HEADER=============================================================
// Firmware for the Arduino managing the low-level controller
//============================================================================================

/////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////

#include "Arduino.h"

#define USB_USBCON
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>


///////////////////////////////////////////////////////////////////
// Init I/O
///////////////////////////////////////////////////////////////////

// ROS
ros::NodeHandle  nodeHandle;

//Publisher 
const int sensors_msg_length = 18;
float sensors_data[ sensors_msg_length ];
std_msgs::Float32MultiArray sensors_msg;
ros::Publisher sensors_pub("sensors", &sensors_msg);

// Serial Communication
const unsigned long baud_rate = 115200;

// Pins for inputs/outputs
// ...
// etc
// ...

///////////////////////////////////////////////////////////////////
// Parameters
///////////////////////////////////////////////////////////////////

// Controller

//Controller parameters here
const float filter_rc  =  50; // ms
// ...
// etc
// ...
 
// Loop period 
const unsigned long time_period_low   = 2;    // 500 Hz for internal PID loop
const unsigned long time_period_high  = 10;   // 100 Hz  for ROS communication
const unsigned long time_period_com   = 1000; // 1000 ms = max com delay (watchdog)

// Hardware min-zero-max range 
const int pwm_min = -511;
const int pwm_zer = 0;
const int pwm_max = 511;

// Units Conversion
const double batteryV  = 24; 
const double volt2pwm   = (pwm_max-pwm_zer)/batteryV;

///////////////////////////////////////////////////////////////////
// Memory
///////////////////////////////////////////////////////////////////

// Inputs
float ctl_ref    = 0;
int ctl_mode     = 0;

// Ouputs
float cmd = 0;

// Controller memory (differentiation, filters and integral actions)
float filter_state   = 0;

// Loop timing
unsigned long time_now       = 0;
unsigned long time_last_low  = 0;
unsigned long time_last_high = 0;
unsigned long time_last_com  = 0; //com watchdog



///////////////////////////////////////////////////////////////////
// Convertion functions
///////////////////////////////////////////////////////////////////

// Convertion function : Cmd --> PWM
double cmd2pwm (double cmd) {
  
  // Scale and offset
  double pwm_d = cmd * volt2pwm + (double) pwm_zer;
  
  // Rounding and conversion
  int pwm = (int) ( pwm_d + 0.5 );
  
  // Saturations
  if (pwm > pwm_max) {
    pwm = pwm_max;
  }
  if (pwm < pwm_min) { 
    pwm = pwm_min;
  }
  
  return pwm;
}


///////////////////////////////////////////////////////////////////
// Set PWM value
///////////////////////////////////////////////////////////////////
void set_pwm( int pwm ){
  
  // Zero cmd
  if ( pwm == 0 ){
    // turn off the led
    digitalWrite(LED_BUILTIN, LOW);

    // set hardware output function here
    // ...
    // etc
    // ...
  }
  
  // Non-zero PWM
  else{
    
    // turn on the LED
    digitalWrite(LED_BUILTIN, HIGH);

    // set hardware output function here
    // ...
    // etc
    // ...
  }
  
}


///////////////////////////////////////////////////////////////////
// Read command from ROS
///////////////////////////////////////////////////////////////////
void cmd_callback ( const geometry_msgs::Twist&  twistMsg ){
  
  ctl_ref  = twistMsg.linear.x; 
  ctl_mode = twistMsg.linear.z;  // 1    or 2   or 3

  time_last_com = millis(); // for watchdog
}

///////////////////////////////////////////////////////////////////
// Controller One tick
///////////////////////////////////////////////////////////////////
void ctl(){

  ////////////////////
  // Sensors 
  ////////////////////
  
  // Retrieve sensors values hardware functions here
  // ...
  // etc
  // ...

  ////////////////////
  // Filters
  ////////////////////
   float alpha = time_period_low / ( time_period_low + filter_rc );
   float read_value     = ctl_ref;
   float filtered_value = alpha * read_value + (1-alpha) * filter_state;

   //Filter memory
   filter_state = filtered_value;

  ////////////////////
  // Controllers
  ////////////////////

  // Default value
  cmd = 0 ;
  
  //////////////////////////////////////////////////////
  if (ctl_mode == 0 ){
    
    // Zero output
    cmd = filtered_value ; //DEBUG CMD = filtered ref

  }
  //////////////////////////////////////////////////////
  else if (ctl_mode == 1 ){

    cmd = ctl_ref;
    
  }
  //////////////////////////////////////////////////////
  else if (ctl_mode == 2 ){

    cmd = 0;

  }
  ///////////////////////////////////////////////////////
  else if (ctl_mode == 3){

    cmd = 0;
    
  }
  ///////////////////////////////////////////////////////
  else if (ctl_mode == 4){
    
    cmd = 0;
    
  }
  ////////////////////////////////////////////////////////
  else {

    cmd = 0;
    
  }
  
  /////////////////////
  // Write output
  ///////////////////
  int pwm    = cmd2pwm( cmd );
  set_pwm(pwm);
  
}


// ROS suscriber
ros::Subscriber<geometry_msgs::Twist> cmd_subscriber("/cmd", &cmd_callback) ;


///////////////////////////////////////////////////////////////////
// Arduino Initialization
///////////////////////////////////////////////////////////////////
void setup(){
  
  // Init I/0 Pins
  // ...
  // etc
  // ...
  
  // Init Communication
  nodeHandle.getHardware()->setBaud(baud_rate); 
  
  // Init ROS
  nodeHandle.initNode();
  nodeHandle.subscribe(cmd_subscriber) ;
  nodeHandle.advertise(sensors_pub)   ;
  
  // Initialize Pwm to neutral
  set_pwm(0);
  
  // Delay
  delay(3000) ;

  // Ros loop event ounce
  nodeHandle.spinOnce();

}


////////////////////////////////////////////////////////////////////
//  Main Control Loop
////////////////////////////////////////////////////////////////////
void loop(){
  
  time_now = millis();

  /////////////////////////////////////////////////////////////
  // Watchdog: stop if no recent communication from ROS
  //////////////////////////////////////////////////////////////

  if (( time_now - time_last_com ) > time_period_com ) {
    
    // All-stop
    ctl_ref  = 0;  //
    ctl_mode = 0;  // 
    
  }

  ////////////////////////////////////////
  // Low-level controller
  ///////////////////////////////////////

  if (( time_now - time_last_low ) > time_period_low ) {
    
    ctl(); // one control tick

    time_last_low = time_now ;
  }

  ////////////////////////////////////////
  // Sync with ROS high-level controller
  ///////////////////////////////////////

  unsigned long dt = time_now - time_last_high;
  if (dt > time_period_high ) {

    // Feedback loop
    sensors_data[0] = 0;
    sensors_data[1] = 0;
    
    // For DEBUG
    sensors_data[2] = ctl_ref;  // set point received by arduino
    sensors_data[3] = ctl_mode; // ctl mode
    sensors_data[4] = cmd; // ctl mode
    sensors_data[7] = (float)( time_now - time_last_com ); // for com debug
    sensors_data[8] = (float)dt;

    // Msg outputs
    sensors_msg.data        = &sensors_data[0];
    sensors_msg.data_length = sensors_msg_length;
    sensors_pub.publish( &sensors_msg );
    
    // Process ROS Events
    nodeHandle.spinOnce();

    time_last_high = time_now ;

  }
}
