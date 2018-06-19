#include<SPI.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

//interupt attachments
int EncoderR_A = 0;  //pin 2
int EncoderR_B = 5;  //pin 18
int EncoderL_A = 1;  //pin 3
int EncoderL_B = 4;  //pin 19
volatile int CountR = 0;
volatile int CountL = 0;
float CountAvg = 0;

//sampling rate
int T = 10; //(millisecond)

//diameters in inch
float EncRatio = 3.400166; //ratio of wheel revolutions to encoder revolutions through calibration
float CircumWheel = 1.372; //(m)

//speeds
float rpmEnc = 0; //(rpm)
float rpmWheel = 0; //(rpm)
float cartSpeed = 0; //(m/s)

//PID control variables
float desired;
float kp = 0; float ki = 0; float kd = 0;
float prev = 0;
float integral;
float time;

//get speed from python variables
int sp;

//Output to motor variables
byte address = 0x00;
int chipsel = 53;

//ros setup
ros::NodeHandle nh;

void messageCb( const geometry_msgs::Twist& msg){
  desired=msg.linear.x;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb );

geometry_msgs::Twist str_msg;
ros::Publisher chatter("actual_speed", &str_msg);



void setup() {
  //setup ISR for encoders
  attachInterrupt(EncoderR_A, ISRR, RISING);
  attachInterrupt(EncoderL_A, ISRL, RISING);

  //setup python serial communication
  pinMode(13, OUTPUT);
  Serial.begin(9600); 
  pinMode(chipsel, OUTPUT);
  SPI.begin();

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter); 
  //Serial.println("Ready");
}

void loop() {
  sp = motor_control(); //call motor control function to get new input speed
  writeToMotor(sp); //write to motor  
  
str_msg.linear.x= cartSpeed;
chatter.publish( &str_msg );

nh.spinOnce();
  
}

void writeToMotor(float sp){
  Serial.println(sp);
    digitalWrite(chipsel, LOW);
      SPI.transfer(address);
      SPI.transfer(sp);
      digitalWrite(chipsel, HIGH);
}

float motor_control(){
  float actual = measure();
  float error = desired-actual;
  float dt = millis()/1000.0-time;
  integral = integral + error*dt;
  float der = (error-prev)/dt;
  

  int control = kp*error + ki*integral + kd*der;
  prev = error;
  time = millis()/1000.0;

  // control boundaries
  //MCP4151 IC is 7-bit: 0-127
  //MCP4151 IC is 8-bit: 0-255
  if (control > 127){
    control = 127;
  }
  if (control < 0){
    control = 0;
  }
   
  return control;
}

float measure(){
  CountR = 0;
  CountL = 0;
  delay(T);
  CountAvg = (CountR+CountL)/2;
  //Serial.print(CountAvg);
  rpmEnc = 60.0*(CountAvg/1024.0)/(T/1000.0);
  rpmWheel = rpmEnc/(EncRatio);
  cartSpeed = rpmWheel*CircumWheel; 
  return cartSpeed;
}
 
//ISR for left and right wheel
void ISRL() {
 CountL++;
}

void ISRR(){
  CountR++;
}

