#include <ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
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

Servo esc;
Servo steer;
ros::NodeHandle nh;
std_msgs::Float32 flFeedback, frFeedback, rlFeedback, rrFeedback, steeringFeedback, manualSteering, manualThrottle;
volatile sensor_msgs::Imu imu_dat;
volatile unsigned long steering_pwm = 0; //0.08789 degrees per microsecond
volatile unsigned long reciever_esc_pwm = 0;
volatile unsigned long reciever_servo_pwm = 0;
unsigned long MIN_pwm = 0;
unsigned long MAX_pwm = 0;
unsigned long MIN_angle = 0;
unsigned long MAX_angle = 0;
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

void moveCb(const ackermann_msgs::AckermannDriveStamped &msg){
  if(!manual_control){
    steering_angle = msg.drive.steering_angle
}
void setup()
{
  steer.attach(2);
  esc.attach(3);
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
  nh.advertise(pub_manual_steering_command);
  nh.advertise(pub_manual_throttle_command);
  nh.advertise(pub_imu);
  nh.subscribe(sub_command);

  /*****IMU Stuff***********/
  bno.setExtCrystalUse(true);
  imu_dat.header.frame_id = "imu";
  /*************************/
  
  /****Add Threads****/
  threads.addThread(steering_read);
  threads.addThread(reciever_esc_read);
  threads.addThread(reciever_servo_read);
  threads.addThread(imu_read);
  threads.addThread(steering_angle_controller);
  /***********************/


/*calibrate steering*/
  steer.write(0);
  MIN_pwm = steering_pwm
  delay(1000);
  steer.write(0);
  delay(1000);
  MAX_pwm = steering_pwm
  compute_steering_parameters();
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
  steeringFeedback.data = map(steering_pwm, MIN_pwm, MAX_pwm, MIN_angle, MAX_angle);
  pub_steering_feedback.publish(&steeringFeedback);
  manualSteering.data = reciever_servo_pwm
  pub_manual_steering_command.pulish();
  
  imupub.publish(&imu_dat);//slows down serial, comment for debugging
  nh.spinOnce();
  currentMillis=millis();
  currentMillis=currentMillis-startMillis;
  delaytime=10-currentMillis-1;
  if(delaytime<0) delaytime=0;
  threads.delay(delaytime);
  
}

void compute_steering_parameters(){
  float range = abs(MAX_pwm - MIN_pwm)*0.08789;
  range = range/2;
  MIN_angle = -1*range;
  MAX_angle = range;
}
void steering_controller(){
  while(1){
  
    if(manual_control){
      steer.writeMicroseconds(reciever_servo_pwm);
    }else{
      
    }
    threads.delay(10); //100 hz loop
  }
  
}
void steering_read(){
  while(1){
  steering_pwm = pulseIn(23, HIGH); //length of pwm pulse
  threads.delay(5); //200 hz loop
  }
}

void reciever_esc_read(){
  while(1){
  reciever_esc_pwm = pulseIn(15, HIGH); //length of pwm pulse
  threads.delay(10); //100 hz loop
  }
}

void reciever_servo_read(){
  while(1){
  reciever_servo_pwm = pulseIn(14, HIGH); //length of pwm pulse
  threads.delay(10); //100 hz loop
  }
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
