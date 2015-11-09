#include "Wire.h"
#include "I2Cdev.h"
#include "configure.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;            // AD0 low = 0x68

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gyro[3];        // [x, y, z]            gyro vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float K_angle_X, K_angular_rate_X, K_position_X, K_velocity_X;
float K_angle_Y, K_angular_rate_Y, K_position_Y, K_velocity_Y;
float position_X, velocity_X, position_Y, velocity_Y;
float angle_Y, angular_rate_Y,angle_X, angular_rate_X;
float gyro_X, acceler_X, gyro_Y, acceler_Y;
float velocity_filter_X, velocity_filter_Y;
float speed_X, speed_Y;
volatile int speed_feedback_1, speed_feedback_2, speed_feedback_3;
int motor1_output, motor2_output, motor3_output;
int speed_feedback_X, speed_feedback_Y;
int position_Need_X, position_Need_Y;
char bufClr = 0;
bool blinkState = false;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  init_data();
  init_IO();
   
  Wire.begin();// join I2C bus (I2Cdev library doesn't do this automatically)
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  Serial.begin(115200);
  Serial2.begin(9600);
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  delay(2);
  
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(10);
  mpu.setYGyroOffset(-15);
  mpu.setZGyroOffset(14);
  mpu.setZAccelOffset(900); // 1688 factory default for my test chip
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {}
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    //Get sensor data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // angle and angular rate unit: radian
    angle_X = ypr[2] + 0.005;                  // 0.017 is center of gravity offset
    angular_rate_X = -((double)gyro[0]/131.0); // converted to radian
    angle_Y = ypr[1] + 0.035;                   // 0.02 is center of gravity offset
    angular_rate_Y = -((double)gyro[1]/131.0); // converted to radian
/*    
    Serial.print("X: ");
    Serial.print(angle_X * RAD_TO_DEG);Serial.print("  ");
    Serial.print(angular_rate_X * RAD_TO_DEG);Serial.print("  ");
    Serial.print("Y: ");
    Serial.print(angle_Y * RAD_TO_DEG);Serial.print("  ");
    Serial.print(angular_rate_Y * RAD_TO_DEG);Serial.print("  ");
    Serial.print("\n");
*/    
    threeWay_to_xoy(speed_feedback_1, speed_feedback_2, speed_feedback_3);
    XY_speed_calculate();	
    xoy_to_threeWay(speed_X, speed_Y);
    Wireless_Control();				//无线控制
    PWM_output(motor1_output, motor2_output, motor3_output);
  
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED, blinkState);
  }
}

void init_IO()
{
  // configure I/O
  pinMode(LED, OUTPUT);
  pinMode(DIR_M1_A, OUTPUT);//
  pinMode(DIR_M1_B, OUTPUT);//
  pinMode(DIR_M2_A, OUTPUT);//
  pinMode(DIR_M2_B, OUTPUT);
  pinMode(DIR_M3_A, OUTPUT);//
  pinMode(DIR_M3_B, OUTPUT);
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(MOTOR_3, OUTPUT);
  pinMode(SPD_PUL_1, INPUT);//
  pinMode(SPD_PUL_2, INPUT);//
  pinMode(SPD_PUL_3, INPUT);//
  
  // configure external interruption
  attachInterrupt(SPD_INT_1, speed_int_1, RISING);
  attachInterrupt(SPD_INT_2, speed_int_2, RISING);
  attachInterrupt(SPD_INT_3, speed_int_3, RISING);
}

void init_data()
{
  K_angle_X =        6.0928;// * 360.0;		//			 365.0
  K_angular_rate_X = 0.3584;// * 250.0;		//			 220.0
  K_position_X =     0.0011704;// * 100.0;		//			 90.0
  K_velocity_X =     0.159467;// * 270.0;		//			 265.0
  
  K_angle_Y =        6.0928;// * 360.0;		//角度	
  K_angular_rate_Y = 0.3584;// * 250.0;		//角速度
  K_position_Y =     0.0011704;// * 100.0;		//位移	
  K_velocity_Y =     0.159467;// * 270.0;		//速度	
}

byte buf_tmp=0;
void Wireless_Control()
{
  position_Need_X = 0;
  position_Need_Y = 0;
  if(Serial2.available())
  {
    buf_tmp=Serial2.read();
  }
  switch(buf_tmp)
  {
    case FORWARD: position_Need_Y = 4;break;
    case BACKWARD: position_Need_Y = -4;break;
    case LEFT: position_Need_X = 4;break;
    case RIGHT: position_Need_X = -4;break;
    case CLOCKWISE: motor1_output -= 15;
		    motor2_output -= 15;
		    motor3_output -= 15;
                    break;
    case ANTICLOCKWISE: motor1_output += 15;
		        motor2_output += 15;
		        motor3_output += 15;
                        break;
    default:break;
  }

  if(++bufClr > 15)
  {
    buf_tmp=0;
    bufClr = 0;
  }		 		
}

/*
任意方向直线运动
将车体坐标系xoy的速度转换成电机的控制坐标系中的速度

			   电机方向			  全局运动坐标	     传感器测量轴

			   /\            |        ^			  |      ^
			V1 /     \ V2    |    Vy  |			  |	   x |
			         \/      |        |			  |      |
			     <--         |         ---->	  |		  ---->
			     V3          |          Vx		  |		   y
*/
void xoy_to_threeWay(float in_X, float in_Y)
{
  motor1_output = in_X * 2 - (1.155 * in_Y);
  motor2_output = in_X * 2 + (1.155 * in_Y);
  motor3_output = -in_X * 2.6; // 2 is motor3 dead zone
}

void threeWay_to_xoy(int in_speed1, int in_speed2, int in_speed3)
{
  speed_feedback_X = (in_speed3-(in_speed2+in_speed1)*0.5)*0.5;
  speed_feedback_Y = -0.866*(in_speed2-in_speed1);
}

//计算XY分量上的速度
void XY_speed_calculate(void)	
{
 /*
  int K_angle_AD = analogRead(K_AGL_AD)-512;
  int K_angular_rate_AD = analogRead(K_AGL_DOT_AD)-512;
  int K_position_AD = analogRead(K_POS_AD)-512;
  int K_velocity_AD = analogRead(K_POS_DOT_AD)-512;
*/
  /******** X轴 *********/
  velocity_X = speed_feedback_X;			//车轮的平均速度
  	
  velocity_filter_X *= 0.95;				//车速低通滤波
  velocity_filter_X += velocity_X * 0.05;
  	
  position_X += velocity_filter_X;		//车轮的位置
  position_X += position_Need_X;  		//期望位移
  	
  if(position_X < -POSITION_LIMIT)		//限幅，防止位置误差过大导致的不稳定
    position_X = -POSITION_LIMIT;
  else if(position_X > POSITION_LIMIT)
    position_X = POSITION_LIMIT;

  speed_X = K_angle_X * angle_X * 69  //69
          + K_angular_rate_X * angular_rate_X * (-124)   // -124
          + K_position_X * position_X * 106   //106
          + K_velocity_X * velocity_filter_X * 90; //90

	/******** Y轴 *********/
  velocity_Y = speed_feedback_Y;			//车轮的平均速度
  	
  velocity_filter_Y *= 0.95;				//车速低通滤波
  velocity_filter_Y += velocity_Y * 0.05;
  	
  position_Y += velocity_filter_Y;  		//车轮的位置
  position_Y += position_Need_Y;  		//期望位移
	
  if(position_Y < -POSITION_LIMIT)		//限幅，防止位置误差过大导致的不稳定
    position_Y = -POSITION_LIMIT;
  else if(position_Y > POSITION_LIMIT)
    position_Y = POSITION_LIMIT;
	
  speed_Y = K_angle_Y * angle_Y * 163   //163 // * K_angle_AD
          + K_angular_rate_Y * angular_rate_Y * 107  //107 // * K_angular_rate_AD
          + K_position_Y * position_Y * 130  //130 // * K_position_AD
          + K_velocity_Y * velocity_filter_Y * 75;  //75 // * K_velocity_AD

/*
  Serial.print(K_angle_AD);Serial.print("  ");
  Serial.print(K_angular_rate_AD);Serial.print("  ");
  Serial.print(K_position_AD);Serial.print("  ");
  Serial.print(K_velocity_AD);Serial.print("  ");
  Serial.print("\n");
*/  
/*   
  Serial.print(motor1_output);Serial.print("  ");
  Serial.print(motor2_output);Serial.print("  ");
  Serial.print(motor3_output);Serial.print("  ");
  
  Serial.print(speed_feedback_1);Serial.print("  ");
  Serial.print(speed_feedback_2);Serial.print("  ");
  Serial.print(speed_feedback_3);Serial.print("  ");
  
  Serial.print(speed_X);Serial.print("  ");
  Serial.print(speed_Y);Serial.print("  ");
  
  Serial.print(speed_feedback_X);Serial.print("  ");
  Serial.print(speed_feedback_Y);Serial.print("  ");
  
  Serial.print("\n");
*/
  speed_feedback_1 = 0;
  speed_feedback_2 = 0;
  speed_feedback_3 = 0;
}

void PWM_output(int pwm_1,int pwm_2,int pwm_3)
{
  // PWM1
  if(pwm_1 < 0)
  {
    digitalWrite(DIR_M1_A, LOW);
    digitalWrite(DIR_M1_B, HIGH);
    pwm_1 = -pwm_1;
  }
  else
  {
    digitalWrite(DIR_M1_A, HIGH);
    digitalWrite(DIR_M1_B, LOW);
  }
  //PWM2
  if(pwm_2 < 0)
  {
    digitalWrite(DIR_M2_A, LOW);
    digitalWrite(DIR_M2_B, HIGH);
    pwm_2 = -pwm_2;
  }
  else
  {
    digitalWrite(DIR_M2_A, HIGH);
    digitalWrite(DIR_M2_B, LOW);
  }
  //PWM3
  if(pwm_3 < 0)
  {
    digitalWrite(DIR_M3_A, LOW);
    digitalWrite(DIR_M3_B, HIGH);
    pwm_3 = -pwm_3;
  }
  else
  {
    digitalWrite(DIR_M3_A, HIGH);
    digitalWrite(DIR_M3_B, LOW);
  }

  pwm_1 += MOTOR_DEADZONE;
  pwm_2 += MOTOR_DEADZONE;
  pwm_3 += MOTOR_DEADZONE;
	
/*
  if(abs(angle_X)>0.7 || abs(angle_Y)>0.7)
  {
    pwm_1 = pwm_2 = pwm_3 = 0;
  }
*/
  analogWrite(MOTOR_1, pwm_1>PWM_LIMIT? PWM_LIMIT:pwm_1);	// Pin6 Motor 1
  analogWrite(MOTOR_2, pwm_2>PWM_LIMIT? PWM_LIMIT:pwm_2);	// Pin7 Motor 2
  analogWrite(MOTOR_3, pwm_3>PWM_LIMIT? PWM_LIMIT:pwm_3);	// Pin8 Motor 3
}

void speed_int_1()
{
  if(digitalRead(SPD_PUL_1)) speed_feedback_1+=1;
  else speed_feedback_1-=1;
}

void speed_int_2()
{
  if(digitalRead(SPD_PUL_2)) speed_feedback_2+=1;
  else speed_feedback_2-=1;
}

void speed_int_3()
{
  if(digitalRead(SPD_PUL_3)) speed_feedback_3+=1;
  else speed_feedback_3-=1;
}

