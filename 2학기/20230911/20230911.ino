#include <MsTimer2.h>

const byte outPin = 13; // Output pin: digital pin 13(D13)
const byte interruptPin1 = 2; // Interrupt pin: D2
const byte interruptPin2 = 2; // Interrupt pin: D2
const byte encoder1_A = 4;
const byte encoder2_B = 5;

const byte resetPin = 5;
volatile byte state = 0;

unsigned long cnt1 = 0;
unsigned long cnt2 = 0;

unsigned long cnt1_old = 0;
unsigned long cnt2_old = 0;


double pulse_to_distance_left = 0.2/512;
double pulse_to_distance_right = 0.2/526;

const double odom_left  = 0;
const double odom_right = 0;

double heading(double x,double y)
{
  double head = atan2(y, x);  // Slope y, S;ope x

  return head;
}


void MsTimer2_ISR()
{
  double odom_left_delta  = 0.0;
  double odom_right_delta = 0.0;
  double theta_delta = 0.0;
  double wheel_track = 0.15;
  
  odom_left_delta  = (cnt1 - cnt1_old) * pulse_to_distance_left;
  odom_right_delta = (cnt2 - cnt2_old) * pulse_to_distance_right;


  theta_delta = heading ( wheel_track, (odom_right_delta - odom_left_delta));
  
  Serial.print(odom_left_delta);  Serial.print(" ");
  Serial.print(odom_right_delta); Serial.print(" ");
  theta_delta = theta_delta * 180.0 / 3.14159;
  Serial.println(theta_delta);  Serial.println(" ");
  
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
   attachInterrupt(digitalPinToInterrupt(interruptPin1), intfunc1, RISING); // Enable interrupt
   attachInterrupt(digitalPinToInterrupt(interruptPin2), intfunc2, RISING); // Enable interrupt
   Serial.begin(115200);
   MsTimer2::set(100,MsTimer2_ISR);
   MsTimer2::start;
}

void intfunc1() // Interrupt function
{
  if (encoder1_A == HIGH)
  {
    cnt1++;
  }
  else
  {
    cnt1++;
  }

}

void intfunc2() // Interrupt function
{
  cnt2++; 
}


void loop()
{


  
  Serial.print(cnt1);   Serial.print(" ");
  Serial.println(cnt2);
  
  if(digitalRead(resetPin) == HIGH)
  {
    cnt1 = 0;
    cnt2 = 0;
  }
  
  delay(1000);
}
