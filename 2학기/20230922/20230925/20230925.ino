#include <MsTimer2.h>

#define THRESHOLD_ANGLE1 15
#define THRESHOLD_ANGLE2 7

#define wheel_track 0.1 // m 단위로 구할 것 0.1 10cm
#define RAD2DEG(x) (x*180.0/3.14159)  // RAD에서 DGREE로
#define DEG2RAD(x) (x*3.14159/180.0)  // DEGREE에서 RAD으로

/////////////////////L298N///////////////////////

#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

///////////////////// IMU //////////////////////
#include <Wire.h>
#include <MPU6050_tockn.h>

#define THRESHOLD_ANGLE1 15
#define THRESHOLD_ANGLE2 7

MPU6050 mpu6050(Wire);

float heading_angle      = 0.0;
float init_heading_angle = 0.0;   // 초기 방향
float target_heading_angle = 0.0;
float heading_angle_error = 0.0;  // error 값

const byte outPin = 13;       // Output pin: digital pin 13(D13)
const byte interruptPin1 = 2; // Interrupt pin: D2
const byte interruptPin2 = 3; // Interrupt pin: D2
const byte encoder1_A = 4;
const byte encoder2_B = 5;

const byte resetPin = 5;
volatile byte state = 0;
unsigned long cnt1 = 0;
unsigned long cnt2 = 0;

unsigned long cnt1_old = 0;
unsigned long cnt2_old = 0;

double pulse_to_distance_left  = 0.2/512;
double pulse_to_distance_right = 0.2/526;

const double odom_left  = 0;
const double odom_right = 0;

double yaw = 0.0;
double yaw_degree = 0.0;

struct pose2D
{
  double x;
  double y;
  double theta;
}my_odom;

double heading(double x,double y)
{
  double head = atan2(y, x);  // Slope y, Slope x

  return head;
}

void motor_R_control(int motor_speed_R) // 모터 A의 속도(speed)제어
{
  if (motor_speed_R >= 0)
  {
    digitalWrite(IN1, HIGH);         //모터의 방향 제어
    digitalWrite(IN2, LOW);
    if(motor_speed_R>=255) motor_speed_R = 255;
    analogWrite(ENA, motor_speed_R); //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    if(motor_speed_R<=-255) motor_speed_R = -255;
    analogWrite(ENA, -motor_speed_R);  
  }
}


  
void motor_L_control(int motor_speed_L) // 모터 A의 속도(speed)제어
{
    if (motor_speed_L >= 0)
    {
      digitalWrite(IN3, LOW);         //모터의 방향 제어
      digitalWrite(IN4, HIGH);
      if(motor_speed_L>=255) motor_speed_L = 255;
      analogWrite(ENB, motor_speed_L); //모터의 속도 제어
    }
    else
    {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      if(motor_speed_L<=-255) motor_speed_L = -255;
      analogWrite(ENB, -motor_speed_L);
    }
}
void MsTimer2_ISR()
{
  char msg[100] = {0x00,};
  
  double odom_left_delta  = 0.0;
  double odom_right_delta = 0.0;
  double odom_center_delta  = 0.0;
  
  long   delta_encoder_left  = 0;
  long   delta_encoder_right = 0;
  
  double theta_delta = 0.0;
  double theta_delta_degree = 0.0;

  delta_encoder_left  = cnt1 - cnt1_old;
  delta_encoder_right = cnt2 - cnt2_old;

  // delta_encoder_left  =  100;
  // delta_encoder_right = -100;
  
  //sprintf(msg, "encoder delta : %3d %3d", delta_encoder_left, delta_encoder_right);
  //Serial.println (msg);
  
  Serial.print("delta_encoder_left  : "); Serial.print(delta_encoder_left );  Serial.print("  ");
  Serial.print("delta_encoder_right : "); Serial.println(delta_encoder_right);
  
  odom_left_delta   = delta_encoder_left  * pulse_to_distance_left;
  odom_right_delta  = delta_encoder_right * pulse_to_distance_right;
  odom_center_delta = (odom_left_delta + odom_right_delta) * 0.5;
  
  Serial.print("odom_left_delta  : "); Serial.print(odom_left_delta );  Serial.print("  ");
  Serial.print("odom_right_delta : "); Serial.println(odom_right_delta);
  
  theta_delta = heading ( wheel_track, (odom_right_delta - odom_left_delta));
  
  Serial.print("delta theta radian"); Serial.println(theta_delta);
  yaw += theta_delta;
  
  theta_delta_degree = RAD2DEG(theta_delta);
  Serial.print("yaw radian : "); Serial.println(yaw);
  
  Serial.print("delta theta degree"); Serial.println(theta_delta_degree);
  yaw_degree += theta_delta_degree;
  Serial.print("yaw degree : ");  Serial.println(yaw_degree);   Serial.println(" ");

  my_odom.x     += odom_center_delta * sin(theta_delta);
  my_odom.y     += odom_center_delta * cos(theta_delta);
  my_odom.theta += theta_delta; 
  Serial.print("odom: ");
  Serial.print(my_odom.x);   ("  ");
  Serial.print(my_odom.y);   ("  ");
  Serial.print(my_odom.theta);
  
/*
  Serial.print("delta_encoder_left  : "); Serial.print(delta_encoder_left );  Serial.print("  ");
  Serial.print("delta_encoder_right : "); Serial.println(delta_encoder_right);

  Serial.print("odom_left_delta  : ");  Serial.print(odom_left_delta );  Serial.print(" ");
  Serial.print("odom_right_delta : ");  Serial.print(odom_right_delta);  Serial.print(" ");
  Serial.print(" w.t. : "); Serial.print(wheel_track); 
  
  Serial.println(theta_delta);  Serial.println(" ");
  Serial.println(theta_delta_degree);  Serial.println(" ");*/
  
  cnt1_old = cnt1;
  cnt2_old = cnt2;
}

void setup()
{
   pinMode(outPin, OUTPUT); // Output mode
   pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
   pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
   pinMode(encoder1_A, INPUT_PULLUP);    // Input mode, pull-up
   pinMode(encoder2_B, INPUT_PULLUP);    // Input mode, pull-up
   pinMode(resetPin, INPUT);
   // attachInterrupt(digitalPinToInterrupt(interruptPin1), intfunc1, RISING); // Enable interrupt
   // attachInterrupt(digitalPinToInterrupt(interruptPin2), intfunc2, RISING); // Enable interrupt
   attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
   attachInterrupt(digitalPinToInterrupt(encoder2_B), intfunc2, RISING); // Enable interrupt

   my_odom.x = 0;
   my_odom.y = 0;
   my_odom.theta = 0;
   
   Serial.begin(115200);
   MsTimer2::set(100,MsTimer2_ISR);
   MsTimer2::start;

   Wire.begin();  // IMU initiallize
}

void intfunc1() // Interrupt function
{
  if (encoder1_A == HIGH)
  {
    cnt1++;
  }
  else
  {
    cnt1--;
  }
}

void intfunc2() // Interrupt function
{
  if (encoder2_B == HIGH)
  {
    cnt2++;
  }
  else
  {
    cnt2--;
  }
}

void loop()
{
  // 전진
  delay(1000);
  // 회전
  delay(1000);
  // 전진
}
