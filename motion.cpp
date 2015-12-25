int v_max = 7.65;
int w_max = 0.45;

#include <math.h>
#include <Timer.h>

boolean flag;
Timer t;

//Define Variables we'll be connecting to
double motor_output_R;
double motor_output_L;

#define encoder_positive 8
#define encoder0PinA  2
#define motorR 11
#define encoder1PinA  3
#define motorL 10

#define motor1_pwm2 6
#define enable 7
#define motor2_pwm2 10

volatile unsigned long encoderR=0, encoderL=0;
unsigned long encoderR_old=0, encoderL_old=0;
int encoderR_dif=0, encoderL_dif=0;

char incomingByte; // for incoming serial data
int get_x_y_new_Event;
int go_to_goal_Event;
double DR = 0, DL =0;
double DC, delta_theta, radius, prev_theta, new_theta;
double x, y, x_new, y_new, x_goal, y_goal;
double Kp, Ki, Kd, prev_err, accum_err;
boolean has_reached_goal = false;

double diff_x, diff_y, distance_error, theta_goal, theta_error;
double Prop_error, Int_error, Dif_error, PID_output;
double omega, velocity, vel_r, vel_l, wheel_radius;
double vel_r_in_pulses, vel_l_in_pulses, vel_rl_max, vel_rl_min;
double vel_Kp, vel_PID_output;

int send_data_event;
int x_goal_n;
int y_goal_n;

void setup() {
  flag = true;

  pinMode(enable , OUTPUT);
  digitalWrite(enable,HIGH);
  pinMode(encoder_positive,OUTPUT);
  digitalWrite(encoder_positive, HIGH);
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resisto
  pinMode(motorR,OUTPUT);
  attachInterrupt(0, doEncoder_0, RISING);  // encoder pin on interrupt 0 - pin 2
  analogWrite(motorR, 0);
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA, HIGH);
  pinMode(motorL,OUTPUT);
  attachInterrupt(1, doEncoder_1, RISING);  // encoder pin on interrupt 1 - pin 3
  analogWrite(motorL, 0);
  analogWrite(motor1_pwm2 , LOW);
  analogWrite(motor2_pwm2 , LOW);

  get_x_y_new_Event = t.every(100, get_x_y_new_direct);

  x_goal = 200;

  y_goal = 0;

  has_reached_goal = false;

  go_to_goal_Event = t.every(100, go_to_goal);

  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps

}

void loop() {
  t.update();
}

void send_data()

{
  Serial.print("x: ");

  Serial.print(x);

  Serial.print(", y: ");

  Serial.print(y);

  Serial.print(", th: ");

  Serial.print(prev_theta*180/3.1418);

  Serial.print(", theta_goal: ");

  Serial.print(theta_goal*180/3.1418);

  Serial.print(", theta_error: ");

  Serial.print(theta_error*180/3.1418);

  Serial.print(", encoderR: ");

  Serial.print(encoderR);

  Serial.print(", encoderL: ");

  Serial.print(encoderL);

  Serial.println();

  if(!flag)
  {
    t.stop(send_data_event);
  }
}

void go_to_goal()

{
  x_goal_n = x_goal - x;
  y_goal_n = y_goal - y;

  diff_x = x_goal - x;
  diff_y = y_goal - y;

  theta_goal = atan2(y_goal_n ,x_goal_n);  //0.785

  theta_error = theta_goal - prev_theta;

  theta_error = atan2(sin(theta_error), cos(theta_error));

  if(theta_error > 0)
  {
    analogWrite(motorR, 85);
    analogWrite(motorL, 0);
  }
  else
  {
    analogWrite(motorR, 0);
    analogWrite(motorL, 85);
  }

  if((abs(diff_x)<5) && (abs(diff_y)<5))
  {
    // stop
    analogWrite(motorR, 0);

    analogWrite(motorL, 0);

    t.stop(go_to_goal_Event);
    t.stop(get_x_y_new_Event);
    flag = false;
  }
}

void get_x_y_new_direct()

{

  encoderR_dif = encoderR - encoderR_old;

  encoderR_old = encoderR;

  encoderL_dif = encoderL - encoderL_old;

  encoderL_old = encoderL;


  DR = 2*3.1418*3 * encoderR_dif/8.0;

  DL = 2*3.1418*3 * encoderL_dif/8.0;

  DC = (DR+DL)/2.0;

  delta_theta = (DL-DR)/13.5;

  x_new = x + DC * cos(prev_theta);

  y_new = y + DC * sin(prev_theta);

  new_theta = prev_theta + delta_theta;

  x = x_new;
  y = y_new;

  prev_theta = new_theta;
  prev_theta = atan2(sin(prev_theta), cos(prev_theta));

}


void doEncoder_0()

{
  encoderR++;
}

void doEncoder_1()

{
  encoderL++;
}
