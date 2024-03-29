//////////////////////////sonar//////////////////////////
#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.   //150
#define WALL_GAP_DISTANCE 260      //mm 단위      //260
#define WALL_GAP_DISTANCE_HALF 190 //mm 단위
#define MOTOR_PWM_OFFSET 10

#define Front 0
#define Left  1
#define Right 2

#define TRIG1 2   // 초음파 센서 1번 Trig 핀 번호 Front
#define ECHO1 3  // 초음파 센서 1번 Echo 핀 번호  Front

#define TRIG2 4   // 초음파 센서 2번 Trig 핀 번호 Left
#define ECHO2 5  // 초음파 센서 2번 Echo 핀 번호  Left

#define TRIG3 6   // 초음파 센서 3번 Trig 핀 번호 Right
#define ECHO3 7  // 초음파 센서 3번 Echo 핀 번호  Right

  NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIG2, ECHO2, MAX_DISTANCE),
  NewPing(TRIG3, ECHO3, MAX_DISTANCE)


};

  float front_sonar  = 0.0;
  float left_sonar   = 0.0;
  float right_sonar  = 0.0;


///////////////////////// L298 /////////////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

///////////////////////// Maze Status /////////////////////////
int maze_status = 0;

void setup() {
  // put your setup code here, to run once:

  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);

  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENB, OUTPUT);
 

  Serial.begin(115200); // 통신속도를 115200으로 정의함
}

void motor_A_control(int direction_a, int motor_speed_a) //모터 A의 방향(direction)과 속도(speed)제어
{
  if(direction_a == HIGH)
  {
    digitalWrite(IN1,HIGH);          //모터의 방향 제어
    digitalWrite(IN2,LOW);
    analogWrite(ENA,motor_speed_a);  //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA,motor_speed_a);
  }
}


void motor_B_control(int direction_b, int motor_speed_b) //모터 B의 방향(direction)과 속도(speed)제어
{
  if(direction_b == HIGH)
  {
    digitalWrite(IN3,HIGH);          //모터의 방향 제어
    digitalWrite(IN4,LOW);
    analogWrite(ENB,motor_speed_b);  //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(ENB,motor_speed_b);
  }
}
void check_maze_status(void)
{
  if( (left_sonar>= 0) && (left_sonar <=WALL_GAP_DISTANCE_HALF  ) && (right_sonar>= 0) && (right_sonar <=WALL_GAP_DISTANCE_HALF  ) && (front_sonar>= 0) && (front_sonar <=WALL_GAP_DISTANCE_HALF  ))

  {
     maze_status = 4;
    Serial.println("maze_status = 4");
  }
  else if( (left_sonar>= 0) && (left_sonar <=WALL_GAP_DISTANCE_HALF  ) && (right_sonar>= 0) && (right_sonar <=WALL_GAP_DISTANCE_HALF  ) && (front_sonar >=WALL_GAP_DISTANCE_HALF  ) )
  {
    maze_status = 1; //왼쪽 오른쪽 가운데
    Serial.println("maze_status = 1");
  }
  else if( (left_sonar>= 0) && (left_sonar <= 200 ) && (front_sonar>= 0) && (front_sonar <=WALL_GAP_DISTANCE_HALF  ) )
  {
     maze_status = 2; // 왼쪽 가운데
    Serial.println("maze_status = 2"); 
  }
   else if( (right_sonar>= 0) && (right_sonar <= 200  ) && (front_sonar>= 0) && (front_sonar <=WALL_GAP_DISTANCE_HALF  ) )
  {
     maze_status = 3; //오른쪽 가운데
    Serial.println("maze_status = 3");
  }
  else
  {
    maze_status = 0;
    Serial.println("maze_status = 0");
  }
}
//먼저 left_pwm =0;  right_pwm =100; 으로 해서 왼쪽 오른쪽 방향 찾기
void wall_collision_avoid(int base_speed)
{
  float error = 0.0;
  float Kp = 0.25;  //0.25       //나중에 조정해야 할 것
  int pwm_conrol = 0; // pwm을 0으로 초기화
  int right_pwm = 0;  // 모터속도 0으로 초기화
  int left_pwm = 0;   // 모터속도 0으로 초기화
 
  error = (right_sonar - left_sonar);
  error = Kp * error;

  if (error >=  40) error =  40; // 35
  if (error <= -35) error = -35; // -30

  right_pwm = (base_speed - error) * 1.8;
  left_pwm  = (base_speed + error) * 2;

  if(right_pwm <= 0) right_pwm = 0;
  if(left_pwm  <= 0) left_pwm  = 0;

  if(right_pwm >= 255) right_pwm = 255;
  if(left_pwm  >= 255) left_pwm  = 255;

  motor_A_control(HIGH,right_pwm); // 오른쪽 전진
  motor_B_control(HIGH,left_pwm);  // 왼쪽 전진

}

void loop()
{
  // put your main code here, to run repeatedly:

  front_sonar = sonar[Front].ping_cm()*10;   // 전방 센서 측정
  left_sonar  = sonar[Left].ping_cm()*10;    // 좌측 센서 측정
  right_sonar = sonar[Right].ping_cm()*10;   // 우측 센서 측정
 
  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE*10;   // 0.0 출력이 최댓값이므로
  if(left_sonar  == 0.0) left_sonar = MAX_DISTANCE*10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE*10;

  Serial.print("L:");Serial.print(left_sonar); Serial.print(" ");
  Serial.print("F:");Serial.print(front_sonar); Serial.print(" ");
  Serial.print("R:");Serial.println(right_sonar);

  check_maze_status();


   if(maze_status == 4)
  {


    //180회전을 한다
    Serial.println("Rotate CCW");
    motor_A_control(LOW,80); // 왼쪽 전진
    motor_B_control(HIGH,125); // 오른쪽 후진
    delay(1290);

  }
  else if( maze_status == 1)
  { // 좌우 벽만 있을 때 직전
    Serial.println("run straight");
    wall_collision_avoid(80);
  }
 
   else if( maze_status == 2) // 왼쪽 가운데
  {  
  // 시계방향 90
    Serial.println("Rotate CCW");
    motor_A_control(LOW,80);    // 오른쪽 후진
    motor_B_control(HIGH,100);  // 왼쪽은 전진
    delay(730);    //800        // 일정한 시간 동안 회전


  }
   else if ( maze_status == 3)
  {

   // 90도 회전 반시계 방향으로 회전
    Serial.println("Rotate CCW");
    motor_A_control(HIGH,120);  // 오른쪽 직진 // 100
    motor_B_control(LOW, 100);  // 왼쪽은 후진  // 100
    delay(630);    //800               // 일정한 시간 동안 회전

  }
   else if ( maze_status == 0)
  {
    Serial.println("Go straight");
    motor_A_control(HIGH,90);  
    motor_B_control(HIGH,110); 
    
  }
  else
  {
    Serial.println("Go straight");
    wall_collision_avoid(80);
  }
  
}
