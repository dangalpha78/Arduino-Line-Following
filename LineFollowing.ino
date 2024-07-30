#include <PID_v1.h>

int sensor1 = A4;
int sensor2 = A3;
int sensor3 = A2;
int sensor4 = A1;
int sensor5 = A0;

int sensor[5] = {0,0,0,0,0};


int ENA = 6;
int IN1 = 7;
int IN2 = 8;
int ENB = 5;
int IN3 = 3;
int IN4 = 4;


double base_speed = 200;
double motor_speed = base_speed;

int banh_chinh = 150;
int banh_phu = 50;
int toc_do_lui = 110;
int left_motor_speed; 
int right_motor_speed; 

double Kp = 16;
double Ki = 0;
double Kd = 11;

int memory = 0;

double max_PID_value = 255 - motor_speed;
double error = 0, PID_value = 0, Setpoint = 0;
PID myPID(&error, &PID_value, &Setpoint, Kp, Ki, Kd, DIRECT); 

void setup() {
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
  delay(500);
  Serial.println("RUN");
  delay(1000);

  myPID.SetOutputLimits(-max_PID_value, max_PID_value);
  myPID.SetMode(AUTOMATIC); 
  myPID.SetSampleTime(20);
}

void loop() {
  read_sensor();
  Serial.println(left_motor_speed);
  Serial.println(right_motor_speed);  
  Serial.println(error);
  if ((error >= -1)&&(error <= 1)) memory = error;

  else if (error == -30)
  {
    do
    {
      ReTrai();
      delay(100);
      read_sensor();
    }
    while (error ==-30);
  }

 else if (error == 30)
  {
    do
    {
      RePhai();
      delay(100);
      read_sensor();
    }
    while (error ==30 );
  }
  // else if (error == 31)
  // {
  //   do
  //   {
  //     DiLui();
  //     delay(100);
  //     read_sensor();
  //   }
  //   while (error == 31 );
  // }
   else if (error == 60)
  {
    do
    {
      DiLui();
      delay(40);
      read_sensor();
    }
    while (error == 60 );
  }

  
  else {
    myPID.Compute();
    motor_control();
    
  }

  
}

void read_sensor() {
  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
  sensor[4] = digitalRead(sensor5);
  
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
  error=5;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))
  error=4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
  error=3;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==0))
  error=2;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0))
  error=1;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=0;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-1;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-2;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-3;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-4;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-5;  
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0))
  error= -5;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
  error=5;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
  error=31;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error = 60;
  else 
    error = memory;    
     }

void motor_control()
{ 
//  left_motor_speed = motor_speed  - PID_value;  //Nếu chạy ngược thì đổi + thành - 
// right_motor_speed = motor_speed + PID_value;  

//   // Giới hạn giá trị xuất xung từ 0 - 255
//   left_motor_speed = constrain(left_motor_speed, 0, 255);   
//   right_motor_speed = constrain(right_motor_speed, 0, 255); 
int a;
a = error / 1;
switch(a){
  case(31):
  DiThang();
  delay(100);
  break;
  case(5):
    DiThang();
    delay(100);
    read_sensor();
    if (error == 60)
    {
    left_motor_speed = 100;
    right_motor_speed = 100;
    DiLui();
    delay(200);
    left_motor_speed = 150;
    right_motor_speed = 100;
    RePhai();
    delay(400);
    break;
    }
    else break;
  case(-5):
    DiThang();
    delay(100);
    read_sensor();
    if (error == 60) {
    left_motor_speed = 100;
    right_motor_speed = 100;
    DiLui();
    delay(200);
    left_motor_speed = 100;
    right_motor_speed = 150;
    ReTrai();
    delay(400);
    break;
    }
    else break;

  case(4):
    left_motor_speed = 130;
    right_motor_speed = 80;
    RePhai();
    delay(40);

    break;
  case(-4):
    left_motor_speed = 80;
    right_motor_speed = 130;
    ReTrai();
    delay(40);

    break;
  case(3):
    left_motor_speed = 110;
    right_motor_speed = 60;
    RePhai();
    delay(30);
    break;
  case(-3):
    left_motor_speed = 60;
    right_motor_speed = 110;
    ReTrai();
    delay(30);


    break;
  case(2):
    left_motor_speed = 120;
    right_motor_speed = 114;
    
    break;
  case(-2):
    left_motor_speed = 114;
    right_motor_speed = 120;
    
    break;
  case(1):
    left_motor_speed = 130;
    right_motor_speed = 127;
    
    break;
  case(-1):
    left_motor_speed = 127;
    right_motor_speed = 130;
    
    break;
  case(0):
    left_motor_speed = 100;
    right_motor_speed = 100;
    break;
      
}
  analogWrite(ENA, right_motor_speed); 
  analogWrite(ENB, left_motor_speed);
  
  
  DiThang();
}

void DiThang()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void DiLui()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, toc_do_lui);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, toc_do_lui);
}

void ReTrai() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, right_motor_speed);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, left_motor_speed);
}
void RePhai() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, right_motor_speed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, left_motor_speed);
}
