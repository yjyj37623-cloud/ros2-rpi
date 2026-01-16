// 引脚配置
#define   Emm42_En_Pin     5   // the number of the Emm42_En_pin
#define   Emm42_Stp_Pin    6   // the number of the Emm42_Stp_pin
#define   Emm42_Dir_Pin    7   // the number of the Emm42_Dir_pin

// 电机参数
#define   MotType         1.8
#define   MStep           16
#define   CW              LOW
#define   CCW             HIGH

// 常见速度和延迟参数
#define   HighSpeed       25
#define   MidSpeed        100
#define   LowSpeed        400
#define   NoDelay         0
#define   SmallDelay      100
#define   LongDelay       500

// 数据类型
#define   ANGLE_T         uint16_t
#define   DIR_T           uint8_t
#define   SPEED_T         uint16_t

// 串口通信设置
#define   Brud            57600


/**************************************************************
***  Arduino和Emm42_V4.0步进闭环的接线：
    * 1. 要先根据说明书安装好Emm42_V4.0步进闭环驱动板到电机上
    * 2. Emm42_V4.0步进闭环驱动板的V+和Gnd接7~28V供电
    * 3. Emm42_V4.0闭环驱动板第一次上电，要先点击Cal选项进行编码器校准
    * 4. Arduino的引脚5接闭环驱动的En引脚（确保闭环驱动屏幕上的En选项选择Hold或者L，默认是Hold）
    * 5. Arduino的引脚6接闭环驱动的Stp引脚
    * 6. Arduino的引脚7接闭环驱动的Dir引脚

*** 注意事项：
    * 1. 要先接好线再通电，不要带电拔插6P的端子插头！！！
    * 2. 购买了非工业套餐（不带光耦隔离）要把Arduino控制板的Gnd和Emm闭环驱动板的Gnd接在一起（共地）
    * 3. 购买了工业套餐（带光耦隔离）要把Arduino控制板的5V接到Emm闭环驱动板的Com引脚上（共阳极接法）

*** 如果你的Arduino控制板是Arduino最小系统，使用USB进行供电的话，注意上电顺序：
    * 上电时，先通闭环驱动板的7~28V供电，再通Arduino控制板的USB供电，避免一些效应造成损坏！！！
    * 断电时，先断Arduino控制板的USB供电，再断闭环驱动板的7~28V供电。
***************************************************************/

uint32_t RoundStep = 360 / MotType * MStep;

// 基本旋转函数，方向、角度和速度
int Round(DIR_T dir, ANGLE_T angle, SPEED_T speed) {
  digitalWrite(Emm42_Dir_Pin, dir);
  uint32_t step = uint32_t(angle * 1.0 / 360 * RoundStep);
  for (int i = 0; i < step; i++) {            
    delayMicroseconds(speed);
    digitalWrite(Emm42_Stp_Pin, !digitalRead(Emm42_Stp_Pin));
    delayMicroseconds(speed);
    digitalWrite(Emm42_Stp_Pin, !digitalRead(Emm42_Stp_Pin));
  }
  return 1;
};

// 转出再原路返回
int goback(DIR_T dir, ANGLE_T angle, SPEED_T speed, SPEED_T mid_delay) {
  int res = Round(dir, angle, speed);
  delay(delay);
  res = Round(!dir, angle, speed);
  return res;
};

// 转一圈
int turnAround(DIR_T dir, SPEED_T speed) {
  int res = Round(dir, 360, speed)  ;
  return res;
};



void setup() {
  // put your setup code here, to run once:

/**********************************************************
***  初始化板载外设
    * 1. 初始化引脚5（接到闭环驱动板的En引脚）为输出模式，输出低电平（0V）
    * 2. 初始化PA6（接到闭环驱动板的Stp引脚）为输出模式，输出低电平（0V）
    * 3. 初始化PA7（接到闭环驱动板的Dir引脚）为输出模式，输出低电平（0V）
**********************************************************/
  pinMode(Emm42_En_Pin , OUTPUT);  digitalWrite(Emm42_En_Pin , LOW);  // initialize the Emm42_En_Pin as an output
  pinMode(Emm42_Stp_Pin, OUTPUT);  digitalWrite(Emm42_Stp_Pin, LOW);  // initialize the Emm42_Stp_Pin as an output
  pinMode(Emm42_Dir_Pin, OUTPUT);  digitalWrite(Emm42_Dir_Pin, LOW);  // initialize the Emm42_Dir_Pin as an output

  Serial.begin(Brud);
  Serial.println("open");
};


void loop() {
  if(Serial.available() > 0) {
    String buf = Serial.readStringUntil('\n');
    // Serial.println(buf);
    if (buf[0] == 'a') {
      int dir = int(buf[1]-'0');  
      int angle = int(buf[2]-'0') * 100 + int(buf[3]-'0') * 10 + int(buf[4]-'0');
      int speed = int(buf[5]-'0') * 100 + int(buf[6]-'0') * 10 + int(buf[7]-'0');
      int res = Round(dir, angle, speed);
      Serial.println("y");  
    }
    else if (buf[0] == 'b') {
      int dir = int(buf[1]-'0');
      int angle = int(buf[2]-'0') * 100 + int(buf[3]-'0') * 10 + int(buf[4]-'0');
      int speed = int(buf[5]-'0') * 100 + int(buf[6]-'0') * 10 + int(buf[7]-'0');
      int delay = int(buf[8]-'0') * 100 + int(buf[9]-'0') * 10 + int(buf[10]-'0');;
      // Serial.println(delay);   
      int res = goback(dir, angle, speed, delay);
      Serial.println("y");  
    }
    else if (buf[0] == 'c') {
      int dir = int(buf[1]-'0');
      int speed = int(buf[2]-'0') * 100 + int(buf[3]-'0') * 10 + int(buf[4]-'0');
      int res = turnAround(dir, speed);
      Serial.println("y");  
    }
    else if (buf[0] == 'q') {
      Serial.println("y"); 
    }
    
    // round
    // if (opt == 'a') {
    //   Serial.print("optA");
    //   char dir = readByte();
    //   char angle1 = readByte();
    //   char angle2 = readByte();
    //   char angle3 = readByte();
    //   char speed1 = readByte();   
    //   char speed2 = readByte();   
    //   Serial.print(dir);
    //   Serial.print(angle1);
    //   Serial.print(angle2);
    //   Serial.print(angle3);
    //   Serial.print(speed1);
    //   Serial.print(speed2);
    //   if (dir != NULL && angle1 != NULL && angle2 != NULL && speed1 != NULL && speed2 != NULL) {
    //     ANGLE_T angle = (ANGLE_T(angle1-'0') * 100 + ANGLE_T(angle2-'0') * 10 + ANGLE_T(angle3-'0')); 
    //     SPEED_T speed = (SPEED_T(speed1-'0') * 10 + SPEED_T(speed2-'0')); 
    //     // ANGLE_T angle = (ANGLE_T(angle1) << 8 | ANGLE_T(angle2) << 0);
    //     // SPEED_T speed = (SPEED_T(speed1) << 8 | SPEED_T(speed2) << 0);
    //     Serial.println(angle, speed);         
    //     if(dir == "r") {
    //       int res = Round(CW, angle, speed);
    //       Serial.println("sucess");          
    //     }     
    //     else {
    //       int res = Round(CCW, angle, speed);
    //       Serial.println("sucess");          
    //     }  
    //   }
    // }
    // // goback
    // else if (opt == 'b') {
    //   char dir = readByte();
    //   char angle1 = readByte();
    //   char angle2 = readByte();
    //   char speed1 = readByte();   
    //   char speed2 = readByte();   
    //   char delay1 = readByte();   
    //   char delay2 = readByte();  
    //   if (dir != NULL && angle1 != NULL && angle2 != NULL && speed1 != NULL && speed2 != NULL && delay1 != NULL && delay2 != NULL) {
    //     DIR_T dirr = DIR_T(dir);
    //     ANGLE_T angle = (ANGLE_T(angle1) << 8 | ANGLE_T(angle2) << 0);
    //     SPEED_T speed = (SPEED_T(speed1) << 8 | SPEED_T(speed2) << 0);
    //     SPEED_T delay = (SPEED_T(delay1) << 8 | SPEED_T(delay2) << 0);          
    //     int res = goback(dir, angle, speed, delay);
    //     Serial.println("sucess");
    //   }
    // }
    // // turn around
    // else if (opt == 'c') {
    //   char dir = readByte();
    //   char speed1 = readByte();   
    //   char speed2 = readByte();   
    //   if (dir != NULL&& speed1 != NULL && speed2 != NULL) {
    //     DIR_T dirr = DIR_T(dir);
    //     SPEED_T speed = (SPEED_T(speed1) << 8 | SPEED_T(speed2) << 0);        
    //     int res = turnAround(dir, speed);
    //     Serial.println("sucess");
    //   }
    // }
    // else {
    //   Serial.println("error");
    // }
  }
}
