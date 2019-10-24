#include "PS2X_lib.h"  //for v1.6
#include "LobotServoController.h"

//ps2手柄
#define PS2_DAT        12  //14    
#define PS2_CMD        11  //15
#define PS2_SEL        10  //16
#define PS2_CLK        13  //17

#define pressures   true
//#define pressures   false
#define rumble      true
//#define rumble      false

PS2X ps2x;
LobotServoController myse(Serial3);

int error = 0;
byte type = 0;
byte vibrate = 0;

#define pwm_max 254

#define base_speed_max 110
#define dir_speed_max 80

#define motor_speed_max 140
#define motor_speed_min 20

#define  pwm_mid 127

#define IN1 8
#define IN2 9
#define ENA 7
#define PWMA 7

#define IN3 4
#define IN4 3
#define ENB 6
#define PWMB 6

void setup() {
  Serial.begin(57600);
  Serial3.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
  digitalWrite(ENA, 0);
  digitalWrite(ENB, 0);
  //校准，获取中间值
  delay(500);

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);


  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
  }

}

void loop() {
  if (error == 1) { //skip loop if no controller found
    //Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
    ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    delay(400);
    return;
  }
  ps2x.read_gamepad(false, vibrate);
  static int speed_a, speed_b, speed_dir , speed_base;
  static int pwm0_t_change = 0;//增加滤波变量
  static int pwm0_t = 0;
  static int pwm1_t_change = 0;//增加滤波变量
  static int pwm1_t = 0;//方向

  pwm0_t_change = ps2x.Analog(PSS_LY);
  pwm1_t_change = ps2x.Analog(PSS_LX);
  //滤波
  if (abs(pwm0_t - pwm0_t_change) > 4) {
    pwm0_t = pwm0_t_change;
  }
  //滤波
  if (abs(pwm1_t - pwm1_t_change) > 4) {
    pwm1_t = pwm1_t_change;
  }

  if (pwm0_t - pwm_mid > 100) {
    speed_base = base_speed_max;
  }
  else if (pwm0_t - pwm_mid < -100) {
    speed_base = -base_speed_max;
  }
  else if(abs(pwm0_t - pwm_mid)<5){
    speed_base = 0;
  }
  else {
    speed_base = map(pwm0_t - pwm_mid, -100 , 100, -base_speed_max , base_speed_max);
  }

  if (pwm1_t - pwm_mid > 70) {
    speed_dir = dir_speed_max;
  }
  else if (pwm1_t - pwm_mid < -70) {
    speed_dir = -dir_speed_max;
  }
  else {
    speed_dir = map(pwm1_t - pwm_mid, -70 , 70, -dir_speed_max , dir_speed_max);
  }

  speed_a = map(speed_base + speed_dir, -dir_speed_max - base_speed_max, dir_speed_max + base_speed_max, motor_speed_max, -motor_speed_max);
  speed_b = map(speed_base - speed_dir, -dir_speed_max - base_speed_max, dir_speed_max + base_speed_max, motor_speed_max, -motor_speed_max);

  //滤波
  if (speed_a < -5 ) speed_a -= 20;
  else if (speed_a > 5) speed_a += 20;
  else speed_a = 0;

  //滤波
  if (speed_b < -5) speed_b -= 20;
  else if (speed_b > 5) speed_b += 20;
  else speed_b = 0;

  A_move(speed_a);
  B_move(speed_b);
  Serial.println(speed_a);
  Serial.println(speed_b);
  static int arm_dir = 1500;   //手臂方向，用右摇杆控制   
  static int arm_joint_0 = 1500;//手臂关节0，用右摇杆控制 
  static int arm_joint_1 = 1500;//手臂关节1，用左前后键控制
  static int arm_joint_2 = 1500;//手臂关节1，用左前后键控制
  static int arm_claw_0 = 1500;//手臂爪子方向，用左前后键控制
  static int arm_claw_1 = 1500;//手臂爪子开合，用左前后键控制，注意范围860,1595
  if(ps2x.Button(PSB_L2))  arm_claw_0 += 60;
  if(ps2x.Button(PSB_R2))  arm_claw_0 -= 60;
  if(ps2x.Button(PSB_L1))  arm_claw_1 += 60;
  if(ps2x.Button(PSB_R1))  arm_claw_1 -= 60;
  if(ps2x.Button(PSB_TRIANGLE))  arm_joint_2 -= 60;
  if(ps2x.Button(PSB_CROSS))  arm_joint_2 += 60;
  if(ps2x.Button(PSB_PAD_UP))  arm_joint_1 -= ps2x.Analog(PSAB_PAD_UP)/5;
  if(ps2x.Button(PSB_PAD_DOWN)) arm_joint_1 += ps2x.Analog(PSAB_PAD_DOWN)/5;
  arm_joint_0+=(ps2x.Analog(PSS_RY)-127)/10*6;
  arm_dir-=(ps2x.Analog(PSS_RX)-127)/10*6;
  myse.moveServo(1,lim(arm_joint_0),1);
  myse.moveServo(2,map(lim(arm_claw_1),400,2600,700,2300),1);
  myse.moveServo(3,lim(arm_joint_1),1);
  myse.moveServo(4,map(lim(arm_claw_1),400,2600,860,1595),1);
  myse.moveServo(5,lim(arm_joint_2),1);
  myse.moveServo(6,lim(arm_dir),1);
  vibrate = 0;


//  Serial.print("arm_dir:");
//  Serial.println(arm_dir);
//   Serial.print("arm_joint_0:");
//  Serial.println(arm_joint_0);
//  Serial.print("arm_joint_1:");
//  Serial.println(arm_joint_1);
//   Serial.print("arm_joint_0:");
//  Serial.println(arm_joint_0);
//  Serial.print("arm_claw_0:");
//  Serial.println(arm_claw_0);
//   Serial.print("arm_claw_1:");
//  Serial.println(arm_claw_1);

  /*
    Serial.print("speed_base");
    Serial.print(speed_base);
    Serial.print("   ");
    Serial.print("speed_dir");
    Serial.println(speed_dir);
  */

  // Serial.println(speed_a);
  /*
    Serial.print(pwm0_t);
    Serial.print("   ");
    Serial.println(pwm1_t);
  */
}

int lim(int &x) {
   if(x<400) {
    vibrate = 255;
    x = 400;
   }
  if(x>2600) {
    vibrate = 255;
    x = 2600;
  }
  else vibrate = 0;
  return x;
}

//A out1 out2
void A_move(int speed) {
  if (speed < 0) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
  }
  else {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
  }
  analogWrite(ENA, abs(speed));
}
//B out3 out4
void B_move(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
  }
  else {
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
  }
  analogWrite(ENB, abs(speed));
}
