#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <avr/interrupt.h>
#include <math.h> // for fmod()

// Pin definitions
#define ngat0 0
#define ngat1 1

#define dir1 6
#define pwm1 5
#define hall1 3
#define stop1 7

#define dir2 10
#define pwm2 11
#define hall2 2
#define stop2 9

// PID and timing variables
float dt1 = 0.01, last_time1;
float dt2 = 0.01, last_time2;
volatile float integral1, previous1, output1 = 0;
volatile float integral2, previous2, output2 = 0;
float kp1, ki1, kd1;
float kp2, ki2, kd2;
float Setpoint1 = 0.0; 
float Setpoint2 = 0.0;  // Initialize Setpoint with 0.0
float Setpoint = 0.0;  // Initialize Setpoint with 0.0

volatile float error1, error2;

volatile int pulse1 = 0;
volatile int pulse2 = 0;

volatile float wheelRps1 = 0;
volatile float wheelRps2 = 0;

volatile bool CO_U = false;

int outputmap1;
int outputmap2;

// ROS Node Handle
ros::NodeHandle nh;
//ros::Publisher pub("info_back", &outputMessage);

// ROS Subscriber Callback Function
void setpointCallback(const std_msgs::Float32MultiArray &msg) {
  // Update Setpoint with the value received from the ROS node
  Setpoint1 = msg.data[0];
  Setpoint2 = msg.data[1];

  //outputMessage.data =  inputMessage.data;
  //pub.publish(&outputMessage);
}

// Define ROS Subscriber
ros::Subscriber<std_msgs::Float32MultiArray> setpointSubscriber("setpoint", &setpointCallback);

// Interrupt Service Routine for reading motor speed
ISR(TIMER1_OVF_vect) {
  TCNT1 = 63036;
  nh.spinOnce();
  CO_U = true;
}

// Count pulses for Motor 1
void cntPuls1() {
  pulse1++;
}

// Count pulses for Motor 2
void cntPuls2() {
  pulse2++;
}

// Motor control function
void driveMotor(int dirpin, int pwmPin, float pwm) {
  if (pwm > 0) {
    digitalWrite(dirpin, 1);
  } else {
    digitalWrite(dirpin, 0);
  }
  analogWrite(pwmPin, abs(abs(pwm) - 255));
}

// Timer initialization function
void timer1Init() {
  /* Reset Timer/Counter1 */
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;

  /* Setup Timer/Counter1 */
  TCCR1B |= (1 << CS11) | (1 << CS10);    // prescale = 64
  TCNT1 = 40536;
  TIMSK1 = (1 << TOIE1);                  // Overflow interrupt enable 
}

// Saturation function to cap values
float saturate(long input, long max) {
  long output;
  if (input >= max) {
    output = max;
  }
  return output;
}

// Setup function
void setup() {
  kp1 = 20;
  ki1 = 2;
  kd1 = 0.15;

  kp2 = 20;
  ki2 = 2;
  kd2 = 0.15;

  last_time1 = 0;
  last_time2 = 0;

  attachInterrupt(ngat0, cntPuls1, RISING);
  attachInterrupt(ngat1, cntPuls2, RISING);

  pinMode(dir1, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm2, OUTPUT);  

  //Serial.begin(57600);

  cli();                                  // Disable global interrupts
  timer1Init();
  sei();                                  // Enable global interrupts

  nh.initNode();                          // Initialize ROS node
  nh.subscribe(setpointSubscriber);       // Subscribe to the "information" topic
}

// Main loop function
void loop() {
  // Process incoming ROS messages
  
  if (CO_U == true) {
    CO_U = false;
    wheelRps1 = (pulse1 / 50.0) / 0.01;
    wheelRps2 = (pulse2 / 50.0) / 0.01;
    pulse1 = 0;
    pulse2 = 0;
    error1 = Setpoint - wheelRps1;
    output1 = pid1(error1);
    error2 = Setpoint - wheelRps2;
    output2 = pid2(error2);
    driveMotor(dir1, pwm1, -Setpoint1);
    driveMotor(dir2, pwm2, Setpoint2);
  }

}
void pr_data() {
  Serial.print("output1: ");
  Serial.print(Setpoint);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("wheelRps1: ");
  Serial.print(wheelRps1);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("output2: ");
  Serial.print(Setpoint);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("wheelRps2: ");
  Serial.print(wheelRps2);
  Serial.print("\t");
  Serial.print("\t");
  Serial.println("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("error1: ");
  Serial.print(error1);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("error2: ");
  Serial.println(error2);
}

// PID control for Motor 1
float pid1(float error1) {
  float proportional1 = error1;
  integral1 += error1 * dt1;
  float derivative1 = (error1 - previous1) / dt1;
  previous1 = error1;
  float output1 = (kp1 * proportional1) + (ki1 * integral1) + (kd1 * derivative1);
  return output1;
}

// PID control for Motor 2
float pid2(float error2) {
  float proportional2 = error2;
  integral2 += error2 * dt2;
  float derivative2 = (error2 - previous2) / dt2;
  previous2 = error2;
  float output2 = (kp2 * proportional2) + (ki2 * integral2) + (kd2 * derivative2);
  return output2;
}
