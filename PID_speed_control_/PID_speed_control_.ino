#include <ros.h>
#include <std_msgs/Int16.h>

int interrupt_a = 2;
int interrupt_b = 3;
int MotorDirection = 8;
int MotorSpeed = 9;

int count=0;
long prev_time = 0;
int Prev_count = 0;
float max_error = 360;
float max_pwm = 255;
float kp = max_pwm/max_error;
float ki = 10;
float kd = 1;
double error;
double lastError;
float PID_PWM;
double intError, derivError;
int wanted_speed;

void ISR_a() {
  int a = digitalRead(interrupt_a);
  int b = digitalRead(interrupt_b);
  if (a != b){
    count++;
  }
  else{
    count--;
  }
}

void ISR_b() {
  int a = digitalRead(interrupt_a);
  int b = digitalRead(interrupt_b);
  if (a == b){
    count++;
  }
  else{
    count--;
  }
}

int PID_compute(int wanted){
  
  // encoder speed calculation
  
  long curr_time = millis();
  float delta_time = ((float) (curr_time-prev_time))/10e3;
  float velocity = (count - Prev_count)/delta_time;

  // PID calculation
  
  error = wanted - velocity;
  intError += error * delta_time;
  derivError = (error - lastError)/delta_time;
 
  double out = kp*error + ki*intError + kd*derivError;              
 
  lastError = error;
  Prev_count = count;
  prev_time = curr_time;
 
  return out;
}

ros::NodeHandle nh;

void pwm_input( const std_msgs::Int16& pwm_msg){
  wanted_speed = pwm_msg.data;
}

ros::Subscriber<std_msgs::Int16> sub("wanted pwm", &pwm_input );
void setup() {
  nh.initNode();
  nh.subscribe(sub);
  pinMode(MotorDirection,OUTPUT);
  pinMode(MotorSpeed,OUTPUT);
  pinMode(interrupt_a,INPUT_PULLUP);
  pinMode(interrupt_b,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt_a),ISR_a,CHANGE);
  attachInterrupt(digitalPinToInterrupt(interrupt_b),ISR_b,CHANGE);


}

void loop() {
  PID_PWM = PID_compute(wanted_speed);
  if (PID_PWM>0){
    digitalWrite(MotorDirection, HIGH);
    analogWrite(MotorSpeed,PID_PWM);
  }else{
    digitalWrite(MotorDirection, LOW);
    PID_PWM = abs(PID_PWM);
    analogWrite(MotorSpeed,PID_PWM);
  }
  
  nh.spinOnce();

}
