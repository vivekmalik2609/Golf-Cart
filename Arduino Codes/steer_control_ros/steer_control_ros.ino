// Including all the libraries required for ROS communication

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


// Defining all the Constant Values

#define mid  580                  //found by physically setting the steering at mid
#define left_comp 695               //found by physically setting the steering at left max
#define right_comp 463                //found by physically setting the steering at right max
#define left 680
#define right 480

// Defining the global variables and pin numbers

const int Enable = 2;               //for the servo to start working
const int InA = 7;                  //inhibit input has to be disabled for the motor to spin
const int InB = 9;                 // the PWM input. 1 for direction 1 254 for direction 2
const int fdbk = A0;                //output of the encoder goes here
double currentpos = 0;                //variable for current position
int desired = mid ;               //setting the desired position as mid
double pos_er = 0;                  //position error
double Outputval;

unsigned long lastTime= 0;
double errSUM, LastERR;
double sampletime = 0.1;
double Kp = 1, Ki = 0, Kd = 2.6;
double kp, ki, kd,a,b;

// Variables for the Steering
byte incdata;
long  incoming[2];
float wheel_angle = 0;
float wheelbase=1.68;                  // wheelbase of the cart in metres
int steering_angle=0;
float angular_velocity=0;
float linear_velocity=0;
float radius; 
float desired_radians=0;

// Setting up the subscriber and publisher
ros::NodeHandle nh;

void messageCb( const geometry_msgs::Twist& msg){
  angular_velocity= msg.angular.z;                  // the callback function of the subscriber
  linear_velocity=msg.linear.x;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb );
geometry_msgs::Twist str_msg;
ros::Publisher chatter("chatter", &str_msg);



void setup() {
  Serial.begin(9600);               //start serial comms
  pinMode(Enable, OUTPUT);            //setting enable, InA, InB in output mode
  pinMode(InA, OUTPUT);
  pinMode(InB, OUTPUT);
  kp = Kp;
  ki = Ki * sampletime;
  kd = Kd / sampletime;
  a = kp * 1;
  b = kp * 100;
  
  // Initiating the ROS Topics
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);                    // testing if the code works or not

}

void loop() {
  
  // maths for converting the angular velocity to steering angle 
  radius = linear_velocity/angular_velocity;
  wheel_angle= atan(wheelbase/radius);
  desired_radians = wheel_angle*14.546;
  desired = desired_radians*57.2958;
  
  //Serial.print(desired);
  currentpos = analogRead(fdbk);          //current postion will be the direct input of the analog pin
  pos_er = desired - currentpos;          //self explanatory


   while (pos_er != 0)
         {
          unsigned long now = millis();
          double timechange = (double)(now - lastTime);


          currentpos = analogRead(fdbk);
          pos_er = desired - currentpos;          //self explanatory
          errSUM += (pos_er * timechange);
          double dErr = (pos_er - LastERR) / timechange;

          Outputval = kp * pos_er + ki*errSUM + kd*dErr;

    
          LastERR = pos_er;
          lastTime = now - sampletime;


          if (currentpos < right_comp | currentpos <right)
              {
                digitalWrite(InA, LOW);
                analogWrite(InB, 254);
              }
          else if (currentpos > left_comp | currentpos >left)
              {
                digitalWrite(InA, LOW);
                analogWrite(InB, 1);
              }


         if ((currentpos >= right_comp) && (currentpos <= left_comp))
              {
                if ((desired >= right_comp) && (desired <= left_comp))
               {
                     if (Outputval > 1)
                      {
                        digitalWrite(Enable, HIGH);
                        digitalWrite(InA, LOW);
                        int val_pve = map(Outputval, a, b, 128, 254);
                        if (val_pve > 254)
                             {
                               analogWrite(InB, 254);

                             } 
                        else
                             {
                              analogWrite(InB, val_pve); 
                             }
                      }

                    if (Outputval < -1)
                      {
                            digitalWrite(Enable, HIGH);

                            digitalWrite(InA, LOW);

                             int val_nve = map(Outputval, -a, -b, 126, 1);
                             if (val_nve < 1)
                               {
                                   analogWrite(InB, 1);
                               }
                            
                             else
                               {
                                  analogWrite(InB, val_nve);
 
                               }
            
                      }

              }
    }
  }
  if (pos_er == 0)
  {
    digitalWrite(Enable, HIGH);

    digitalWrite(InA, HIGH);
    analogWrite(InB, 127);   
    
  }

// Publishing the values for testing the code
  str_msg.linear.x= linear_velocity;
  str_msg.angular.z=desired;
  chatter.publish( &str_msg );
  
nh.spinOnce();

    
}



