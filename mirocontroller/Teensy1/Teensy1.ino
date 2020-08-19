#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "TeensyThreads.h"
#define PI 3.1415926535897932384626433832795
#define USE_TEENSY_HW_SERIAL
//ThreadWrap(Serial, SerialX);
//#define Serial ThreadClone(SerialX)
/**********Function Prototypes**************/
void moveCb(const ackermann_msgs::AckermannDriveStamped &msg);
void compute_steering_parameters();
void vehicle_controller();
void steering_read();
void reciever_esc_read();
void reciever_servo_read();
void imu_read();
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void steering_isr();
void rec_servo_isr();
void rec_esc_isr();
 /***********************************/
 //TODO add service for switching to computer control
 /************VARIABLES********************/
Encoder fl(5, 6);
Encoder fr(7, 8);
Encoder rl(9, 10);
Encoder rr(11, 12);

Servo esc;
Servo steer;
ros::NodeHandle nh;
std_msgs::Float32 flFeedback, frFeedback, rlFeedback, rrFeedback, steeringFeedback, manualSteering, manualThrottle;
volatile sensor_msgs::Imu imu_dat;
volatile unsigned long steering_pwm = 0; //0.351564 degrees per microsecond
volatile unsigned long reciever_esc_pwm = 1512;
volatile unsigned long reciever_servo_pwm = 1523;
unsigned long MIN_pwm = 476; //range of PWM is about 112 microseconds
unsigned long MAX_pwm = 581;
unsigned long MIN_angle = 0;
unsigned long MAX_angle = 37;
volatile unsigned long esc_micros = 0;
volatile unsigned long servo_micros = 0;
volatile unsigned long steering_micros = 0;
volatile float steering_angle = 0;

bool manual_control = true;

ros::Publisher pub_fl_feedback("truck/feedback/wheel/fl", &flFeedback);
ros::Publisher pub_fr_feedback("truck/feedback/wheel/fr", &frFeedback);
ros::Publisher pub_rl_feedback("truck/feedback/wheel/rl", &rlFeedback);
ros::Publisher pub_rr_feedback("truck/feedback/wheel/rr", &rrFeedback);
ros::Publisher pub_steering_feedback("truck/feedback/steering", &steeringFeedback);
ros::Publisher pub_manual_steering_command("truck/manual/steering", &manualSteering);
ros::Publisher pub_manual_throttle_command("truck/manual/throttle", &manualThrottle);
ros::Publisher pub_imu("truck/imu/data", &imu_dat);

ros::Subscriber<ackermann_msgs::AckermannDriveStamped> sub_command("truck/in/command", &moveCb); //statically allocated for rosserial using templates

Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);

volatile sensors_event_t orientationData , angVelocityData , linearAccelData;
/************************************************/


void setup(){
  steer.attach(2);
  esc.attach(3);
  //pinMode(14,INPUT_PULLDOWN);
  //pinMode(15,INPUT_PULLDOWN);
  //pinMode(23,INPUT_PULLDOWN);
  
  
  if(!bno.begin())
  {
   //if not detected, infinite loop
    while(1);
  }
  delay(1000);//wait for imu
  //nh.getHardware()->setBaud(256000);
  nh.initNode();
  nh.advertise(pub_fl_feedback);
  nh.advertise(pub_fr_feedback);
  nh.advertise(pub_rl_feedback);
  nh.advertise(pub_rr_feedback);
  nh.advertise(pub_steering_feedback);
  nh.advertise(pub_manual_steering_command);
  nh.advertise(pub_manual_throttle_command);
  nh.advertise(pub_imu);
  nh.subscribe(sub_command);

  /*****IMU Stuff***********/
  bno.setExtCrystalUse(true);
  imu_dat.header.frame_id = "imu";
  /*************************/
  
  

  
  attachInterrupt(23,steering_isr,CHANGE); //steering encoder pwm
/*calibrate steering*/
  steer.write(0);
  delay(1000);
  MIN_pwm = steering_pwm;
  steer.write(180);
  delay(1000);
  MAX_pwm = steering_pwm;
  compute_steering_parameters();
  delay(100);

  attachInterrupt(14,rec_esc_isr,CHANGE); //reciever esc pwm
  attachInterrupt(15,rec_servo_isr,CHANGE); //reciever servo pwm
  /****Add Threads****/
  threads.addThread(imu_read);
  threads.addThread(vehicle_controller);
  /***********************/
}

unsigned long startMillis; //used for timing of ros rate
unsigned long currentMillis;
int delaytime;

void loop()
{
  startMillis=millis();
  flFeedback.data = fl.read();
  pub_fl_feedback.publish(&flFeedback);
  frFeedback.data = fr.read();
  pub_fr_feedback.publish(&frFeedback);
  rlFeedback.data = rl.read();
  pub_rl_feedback.publish(&rlFeedback);
  rrFeedback.data = rr.read();
  pub_rr_feedback.publish(&rrFeedback);
  steeringFeedback.data = mapf(steering_pwm, MIN_pwm, MAX_pwm, -37, 37);
  pub_steering_feedback.publish(&steeringFeedback);
  manualSteering.data = mapf(reciever_servo_pwm,1065,2006,0,100);
  pub_manual_steering_command.publish(&manualSteering);
  manualThrottle.data = mapf(reciever_esc_pwm,1065,2006,0,100);
  pub_manual_throttle_command.publish(&manualThrottle);
  imu_dat.orientation.x = orientationData.orientation.x;
  imu_dat.orientation.y = orientationData.orientation.y;
  imu_dat.orientation.z = orientationData.orientation.z;
  imu_dat.angular_velocity.x = angVelocityData.gyro.x;
  imu_dat.angular_velocity.y = angVelocityData.gyro.y;
  imu_dat.angular_velocity.z = angVelocityData.gyro.z;
  imu_dat.linear_acceleration.x = linearAccelData.acceleration.x;
  imu_dat.linear_acceleration.y = linearAccelData.acceleration.y;
  imu_dat.linear_acceleration.z = linearAccelData.acceleration.z;
  imu_dat.header.stamp = nh.now();
  pub_imu.publish(&imu_dat);//slows down serial, comment for debugging
  nh.spinOnce();
  currentMillis=millis();
  currentMillis=currentMillis-startMillis;
  delaytime=10-currentMillis-1;
  if(delaytime<0) delaytime=0;
  threads.delay(delaytime);
  
}

void moveCb(const ackermann_msgs::AckermannDriveStamped &msg){
  if(!manual_control){
    steering_angle = msg.drive.steering_angle;
    }
}

void compute_steering_parameters(){
  float range = abs(MAX_pwm - MIN_pwm)*0.351564;
  MIN_angle = 0;
  MAX_angle = range;
}
void vehicle_controller(){
  while(1){
  
    if(manual_control){
      steer.writeMicroseconds(reciever_servo_pwm);
      esc.writeMicroseconds(reciever_esc_pwm);
    }else{
      
    }
    threads.delay(10); //100 hz loop
  }
}



void imu_read(){
  while(1){
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    threads.delay(10); //100 hz loop
  }
}
void rec_esc_isr(){
  if(digitalRead(14) == HIGH){
    esc_micros = micros();
  }else{
    if(esc_micros > 0){
      if(micros() > esc_micros && 1000 < (micros() - esc_micros) && 2010 > (micros() - esc_micros)){
        reciever_esc_pwm = micros() - esc_micros;
      }
    }
  }
}

void rec_servo_isr(){
  if(digitalRead(15) == HIGH){
    servo_micros = micros();
  }else{
    if(servo_micros > 0){
      if(micros() > servo_micros && 1000 < (micros() - servo_micros) && 2010 > (micros() - servo_micros)){
        reciever_servo_pwm = micros() - servo_micros;
      }
    }
  }
}

void steering_isr(){
  if(digitalRead(23) == HIGH){
    steering_micros = micros();
  }else{
    if(steering_micros > 0){
      if(micros() > steering_micros){
        steering_pwm = micros() - steering_micros;
      }
    }
  }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
