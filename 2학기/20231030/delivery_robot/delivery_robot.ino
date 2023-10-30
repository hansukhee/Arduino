#define AOpin  A0    // Analog out pin - yellow
#define SIpin  22    // Start Integration - Orange
#define CLKpin 23
#define NPIXELS 128  // NO. of pixels in array
#define BTSerial Serial2
#define RxD 2
#define TxD 3
#define BT_BAUDRATE 9600

double kp_vision = 0.1;
double kd_vision = 0.3;
double ki_vision = 0.0;

double error     = 0.0;
double error_old   = 0.0;
double target    = NPIXELS / 2;
double error_error_old = 0;

byte pixel[NPIXELS];  // Sield for measured values <0 - 255>
byte LineSensor_threshold_Data[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr)  |= _BV(bit))

////////////////////////////////L298N/////////////////////////////

#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

/////////////////////////////////////////////////////////////////////

void setup() 
{ // put your setup code here, to run once:
  int i;

  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023;
    MIN_LineSensor_Data[i] = 0;
  }
  
  pinMode(SIpin,  OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode(AOpin,  OUTPUT);

  digitalWrite(SIpin, LOW);
  digitalWrite(CLKpin, LOW);


  #if FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  #endif
  
  flag_line_adapation = 0;
  
  Serial.begin(115200);
  BTSerial.begin(BT_BAUDRATE);

}

void threshold_line_image(int threshold_value)
{
  for (int i = 0; i < NPIXELS; i++)
  {
    if(pixel[i] >= threshold_value)
    {
       LineSensor_threshold_Data[i] = 255;
    }
    else
    {
      LineSensor_threshold_Data[i] = 0;
    }
  }
}

void motor_control_right(int motor_speed_right) // 모터 A의 속도(speed)제어
{
  if (motor_speed_right >= 0)
  {
    digitalWrite(IN1, HIGH);         //모터의 방향 제어
    digitalWrite(IN2, LOW);
    if(motor_speed_right >=255) motor_speed_right = 255;
    analogWrite(ENA, motor_speed_right); //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    if(motor_speed_right<=-255) motor_speed_right = -255;
    analogWrite(ENA, -motor_speed_right);
  }
}


  
void motor_control_left(int motor_speed_left) // 모터 A의 속도(speed)제어
{
  if (motor_speed_left >= 0)
  {
    digitalWrite(IN3, LOW);         //모터의 방향 제어
    digitalWrite(IN4, HIGH);
    if(motor_speed_left >=255) motor_speed_left = 255;
    analogWrite(ENB, motor_speed_left); //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    if(motor_speed_left <=-255) motor_speed_left = -255;
    analogWrite(ENB, -motor_speed_left);
  }
}

void read_line_camera(void) 
{
  int i;
  delay(1);
  
  digitalWrite(CLKpin, LOW);
  digitalWrite(SIpin, HIGH);
  digitalWrite(CLKpin, HIGH);
  digitalWrite(SIpin,  LOW);
  delayMicroseconds (1);
  
  for (i = 0; i < NPIXELS; i++)
  {
    pixel[i] = analogRead(AOpin)/4 ;
    digitalWrite(CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite(CLKpin, HIGH);
  }
}

double line_centroid(void)
{
  double centroid = 0.0;
  double mass_sum = 0;
  
  for(int i = 0; i < NPIXELS; i++)
  {
    mass_sum += LineSensor_threshold_Data[i];
    centroid += LineSensor_threshold_Data[i] + i;
  }
  centroid = centroid / mass_sum;

  return centroid;
}

int  PID_control(double line_center)
{

  int pwm_value = 0;
  double error_old = 0;
  error = target - line_center;
  error_old = error_error_old;
  pwm_value = int(kp_vision * error + kd_vision + error_old);
  if(pwm_value >= 200)  pwm_value =  200;
  if(pwm_value <= -200) pwm_value = -200;
  error_old = error;

  return pwm_value;
}

void vision_line_control(int base_speed, double l_c)
{
  int pwm_control_value = PID_control(l_c);
  motor_control_left (base_speed + pwm_control_value);
  motor_control_right(base_speed - pwm_control_value);
}

void loop()
{
  double c_x = 0;
  int i;
  
  read_line_camera();
  threshold_line_image(150);
  c_x = line_centroid();
  vision_line_control(50, c_x);

  for(i = 0; i < NPIXELS; i++);
  {
    Serial.println(LineSensor_threshold_Data[i]);
  }
  Serial.println(c_x);
  
}
