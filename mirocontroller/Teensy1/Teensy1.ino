#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "TeensyThreads.h"
#define PI 3.1415926535897932384626433832795
#define USE_TEENSY_HW_SERIAL
//ThreadWrap(Serial, SerialX);
//#define Serial ThreadClone(SerialX)
/**********Function Prototypes**************/

 /***********************************/
 
 /************VARIABLES********************/
Encoder fl(5, 6);
Encoder fr(7, 8);
Encoder rl(9, 10);
Encoder rr(11, 12);

ros::NodeHandle nh;
std_msgs::Float32 flFeedback, frFeedback, rlFeedback, rrFeedback, steeringFeedback, manualSteering, manualThrottle;
ackermann_msgs::AckermannDrive moveCommand
volatile sensor_msgs::Imu imu_dat;
volatile unsigned long steering_pwm = 0;
volatile unsigned long reciever_esc_pwm = 0;
volatile unsigned long reciever_servo_pwm = 0;

ros::Publisher pub_fl_feedback("truck/feedback/wheel/fl", &flFeedback);
ros::Publisher pub_fr_feedback("truck/feedback/wheel/fr", &frFeedback);
ros::Publisher pub_rl_feedback("truck/feedback/wheel/rl", &rlFeedback);
ros::Publisher pub_rr_feedback("truck/feedback/wheel/rr", &rrFeedback);
ros::Publisher pub_steering_feedback("truck/feedback/steering", &steeringFeedback);
ros::Publisher pub_manual_steering_feedback("truck/manual/steering", &manualSteering);
ros::Publisher pub_manual_throttle_feedback("truck/manual/throttle", &manualThrottle);
ros::Publisher pub_imu("truck/imu/data", &imu_dat);

ros::Subscriber<ackermann_msgs::AckermannDrive> sub_command("truck/in/command", &moveCommand); //statically allocated for rosserial using templates

Adafruit_BNO055 bno = Adafruit_BNO055(55,0x28);

volatile sensors_event_t orientationData , angVelocityData , linearAccelData;
/************************************************/
void setup()
{
  pinMode(2, OUTPUT);//servo pwm
  pinMode(3, OUTPUT);//esc pwm
  pinMode(14,INPUT); //reciever esc pwm
  pinMode(15,INPUT); //reciever servo pwm
  pinMode(23,INPUT); //steering encoder pwm
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
  nh.advertise(pub_imu);
  nh.subscribe(sub_command);

  /*****IMU Stuff***********/
  bno.setExtCrystalUse(true);
  imu_dat.header.frame_id = "imu";
  /*************************/
  
  /****Threads Start********/
  threads.addThread(steering_read);
  threads.addThread(reciever_esc_read);
  threads.addThread(reciever_servo_read);
  threads.addThread(imu_read);
  threads.addThread(steering_angle_controller);
  /***********************/
}

unsigned long startMillis; //used for timing of ros rate
unsigned long currentMillis;
int delaytime;
void loop()
{
  startMillis=millis();
  flFeedback.data = fl.read();
  frFeedback.data = fr.read();
  rlFeedback.data = rl.read();
  rrFeedback.data = rr.read();
  
  imupub.publish(&imu_dat);//slows down serial, comment for debugging
  nh.spinOnce();
  currentMillis=millis();
  currentMillis=currentMillis-startMillis;
  delaytime=10-currentMillis-1;
  if(delaytime<0) delaytime=0;
  threads.delay(delaytime);
}

void steering_read(){
  steering_pwm = pulseIn(23, HIGH); //length of pwm pulse
  threads.delay(5); //200 hz loop
}

void reciever_esc_read(){
  reciever_esc_pwm = pulseIn(15, HIGH); //length of pwm pulse
  threads.delay(10); //100 hz loop
}

void reciever_servo_read(){
  reciever_servo_pwm = pulseIn(14, HIGH); //length of pwm pulse
  threads.delay(10); //100 hz loop
}

void steering_angle_controller(){//ISR for front steering servo
  while(1){
    /*
  if(abs(myEnc.read())>1400){ //for max and min turning angle
    analogWrite(11, 0);
    return;
  }*/
    frErr=frSp-front.read();
    if(frErr>=0){
      digitalWrite(1, LOW);
    }else{
      digitalWrite(1, HIGH);
      frErr=frErr*(-1);
    }
    if(frErr*100>60000){
      analogWriteFrequency(0, 60000);
    }else{
      analogWriteFrequency(0, frErr*100);
    }
    analogWrite(0, 200);
    threads.delay(5); 
  }
}


void imu_read(){
  while(1){
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
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
    threads.delay(10); //100 hz loop
  }
}
