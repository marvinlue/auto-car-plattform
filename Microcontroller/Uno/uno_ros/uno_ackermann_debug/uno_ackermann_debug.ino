#include <ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/UInt16.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <Servo.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

Servo esc;
Servo servo;

class Ackermann
{
public:
  Ackermann()
  :subscriber_("Ackermann_Drive", &Ackermann::set_period_callback, this),
  anglePublisher_("PWM_Angle", &angle_msg),
  speedPublisher_("PWM_Speed", &speed_msg)
  {}

  void init(ros::NodeHandle& nh)
  {
    servo.attach(anglePin, 50, 2000);
    servo.writeMicroseconds(generate_pwm_angle(neutralAngle));
    
    esc.attach(velPin);
    esc.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
    delay(2000); // delay to allow the ESC to recognize the stopped signal.

    nh.subscribe(subscriber_);
    nh.advertise(anglePublisher_);
    nh.advertise(speedPublisher_);
  }

  void run(ros::NodeHandle& nh){
    ros::Time current_time = nh.now();
    if (checkMsgTime(current_time)){
      float angle = generate_pwm_angle(velocities_.steering_angle);
      float vel = generate_pwm_velocity(velocities_.speed);
      publish(angle, vel);
      setSignals(angle, vel);
    }
    else {
      float angle = generate_pwm_angle(neutralAngle);
      float vel = generate_pwm_velocity(neutralVel);
      publish(angle, vel);
      setSignals(angle, vel);
    }
  }

  void set_period_callback(const ackermann_msgs::AckermannDriveStamped& msg)
  {
    velocities_ = msg.drive;
    lastAckerMsg = msg.header.stamp;
  }

  void publish(float angle, float vel){
    angle_msg.data = angle;
    anglePublisher_.publish(&angle_msg);
    delay(5);
    speed_msg.data = vel;
    speedPublisher_.publish(&speed_msg);
    delay(5);
  }

  void setSignals(float angle, float vel){
    esc.writeMicroseconds(vel);
    servo.writeMicroseconds(angle);
  }

  bool checkMsgTime (ros::Time currentTime)
  {
    
    double diff = (currentTime.toSec() - lastAckerMsg.toSec());
    float current = currentTime.toSec();
    float msgtime = lastAckerMsg.toSec();
    angle_msg.data = current;
    anglePublisher_.publish(&angle_msg);
    delay(5);
    angle_msg.data = msgtime;
    anglePublisher_.publish(&angle_msg);
    delay(5);
    angle_msg.data = diff;
    anglePublisher_.publish(&angle_msg);
    delay(5);
    return (diff <= 1.0) ;
  }
    

  float generate_pwm_angle (float input)
  {
    float const_angle = constrain(input, minInAngle, maxInAngle);
    if (const_angle >= 0){
      float pwm_value = neutralPWMAngle - (const_angle/maxInAngle) * maxOutDifAngle;
      return pwm_value;
    }
    else {
      float pwm_value = neutralPWMAngle + (const_angle/minInAngle) * minOutDifAngle;
      return pwm_value;
    }
  }

  float generate_pwm_velocity (float input)
  {
    float const_vel = constrain(input, minInVel, maxInVel);
    float pwm_value = neutralPWMVel + minOutDifVel + (const_vel/maxInVel) * maxOutDifVel;
    return pwm_value;
  }

private:
  ros::Subscriber<ackermann_msgs::AckermannDriveStamped, Ackermann> subscriber_;
  ackermann_msgs::AckermannDrive velocities_;
  ros::Publisher anglePublisher_;
  ros::Publisher speedPublisher_;
  std_msgs::Float32 angle_msg;
  std_msgs::Float32 speed_msg;

  ros::Time lastAckerMsg;
  
  int velPin = 3;
  int anglePin = 6;

  int neutralAngle = 0;
  int neutralPWMAngle = 200;
  float minInAngle = -27;
  float maxInAngle = 20;
  float minOutDifAngle = 60;
  float maxOutDifAngle = 50;
  
  int neutralVel = 0; 
  int neutralPWMVel = 1500;
  float minInVel = 0;
  float maxInVel = 15;
  float minOutDifVel = 20;
  float maxOutDifVel = 180;
  
};

Ackermann ackermann;


unsigned long previousMillis = 0;
const long interval = 500; 

void setup()
{
  nh.initNode();
  ackermann.init(nh);
}
  

void loop()
{
unsigned long currentMillis = millis();

if (currentMillis - previousMillis >= interval) {
  nh.requestSyncTime();
  previousMillis = currentMillis;
  ackermann.run(nh);
  nh.spinOnce();
  delay(1);
}
}
