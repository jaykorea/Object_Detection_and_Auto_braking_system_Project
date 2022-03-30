/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif

#include <DynamixelWorkbench.h>

DynamixelWorkbench dxl_wb;

#define BAUDRATE  57600
#define DXL_ID    1
#define MODE 2
#define BDPIN_BUZZER 31
#define USER1_LED 22
#define USER2_LED 23
#define LED_BUILTIN 13
#define BDPIN_PUSH_SW_2 35

uint8_t dxl_id = DXL_ID;
uint8_t mode = MODE;
uint16_t model_number = 0;
bool onetimecall = true;
int val;

#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle  nh;


void object_depth_data(const std_msgs::UInt16MultiArray& msg);

ros::Subscriber<std_msgs::UInt16MultiArray> sub("ProcessedData", object_depth_data);

#define PersonDetectFlag 1
#define PersonProbability 50
#define PersonDistance 1500
#define PersonDistanceLimit 700

uint32_t PersonDistanceLimitCount;

void setup(){
  pinMode(USER1_LED, OUTPUT);
  pinMode(USER2_LED, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);

  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);

  const char *log;
  bool result = false;

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
//    Serial.println(log);
//    Serial.println("Failed to init");
    user2_ledoff();
  }
  else
  {
//    Serial.print("Succeeded to init : ");
//    Serial.println(BAUDRATE);
    user2_ledon();  
  }

  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
//    Serial.println(log);
//    Serial.println("Failed to ping");
    user2_ledoff();
  }
  else
  {
//    Serial.println("Succeeded to ping");
//    Serial.print("id : ");
//    Serial.print(dxl_id);
//    Serial.print(" model_number : ");
//    Serial.println(model_number);
    user2_ledon();
  }

  switch (mode)
  {
    case 0:
      result = dxl_wb.setCurrentControlMode(dxl_id, &log);
      if (result == false)
      {
//        Serial.println(log);
//        Serial.println("Failed to set mode");
        user2_ledoff();
      }
      else
      {
//        Serial.println("Succeeded to set mode");
          user2_ledon();
      }
     break;

    case 1:
      result = dxl_wb.setVelocityControlMode(dxl_id, &log);
      if (result == false)
      {
//        Serial.println(log);
//        Serial.println("Failed to set mode");
          user2_ledoff();
      }
      else
      {
//        Serial.println("Succeeded to set mode");
          user2_ledon();
      }
     break;

    case 2:
      result = dxl_wb.jointMode(dxl_id, 0, 0, &log);
      if (result == false)
      {
//        Serial.println(log);
//        Serial.println("Failed to set mode");
          user2_ledoff();
      }
      else
      {
//        Serial.println("Succeeded to set mode");
          user2_ledon();
      }
     break;

    case 3:
      result = dxl_wb.setExtendedPositionControlMode(dxl_id, &log);
      if (result == false)
      {
//        Serial.println(log);
//        Serial.println("Failed to set mode");
          user2_ledoff();
      }
      else
      {
//        Serial.println("Succeeded to set mode");
          user2_ledon();
      }
     break;

    case 4:
      result = dxl_wb.setCurrentBasedPositionControlMode(dxl_id, &log);
      if (result == false)
      {
//        Serial.println(log);
//        Serial.println("Failed to set mode");
          user2_ledoff();
      }
      else
      {
//        Serial.println("Succeeded to set mode");
          user2_ledon();
      }
     break;

    case 5:
      result = dxl_wb.setPWMControlMode(dxl_id, &log);
      if (result == false)
      {
//        Serial.println(log);
//        Serial.println("Failed to set mode");
          user2_ledoff();
      }
      else
      {
//        Serial.println("Succeeded to set mode");
          user2_ledon();
      }
     break;

    default:
      result = dxl_wb.setPositionControlMode(dxl_id, &log);
      if (result == false)
      {
//        Serial.println(log);
//        Serial.println("Failed to set mode");
          user2_ledoff();
      }
      else
      {
//        Serial.println("Succeeded to set mode");
          user2_ledon();
      }
     break;
  }
  
  dxl_wb.goalPosition(dxl_id, (int32_t)0);
  delay(100);

  nh.initNode();
  nh.subscribe(sub);
  
}

void loop(){
  releasebuttoncheck();
  nh.spinOnce();
  delay(1);
}

void object_depth_data( const std_msgs::UInt16MultiArray& msg){
  
  //servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  if ( (msg.data[0] == PersonDetectFlag) && (msg.data[1] > PersonProbability) && (msg.data[2] < PersonDistance) )
  {
     user1_ledon();
     buzzeron();
     if ( msg.data[2] < PersonDistanceLimit ) {
      //TODO : control motor if specific time over
      //TODO : save the msg.data to Array and calculate average data to apply to the algorithm
      //PersonDistanceLimitCount++;
     if (onetimecall == true) {
      onetimecall = false;
      dxl_wb.goalPosition(dxl_id, (int32_t)341); //30 degree
      delay(100);
     }
    }      
  }
  else { 
    user1_ledoff();
    buzzeroff();
  }
}

void user1_ledon()
{
  digitalWrite(USER1_LED, LOW);  //toggle led 
}

void user1_ledoff()
{
  digitalWrite(USER1_LED, HIGH);
}

void user2_ledon()
{
  digitalWrite(USER2_LED, LOW);
}

void user2_ledoff()
{
  digitalWrite(USER2_LED, HIGH);
}

void builtin_ledon()
{
  digitalWrite(LED_BUILTIN, LOW);
}

void builtin_ledoff()
{
  digitalWrite(LED_BUILTIN, HIGH);
}

void buzzeron()
{
  tone(BDPIN_BUZZER, 1000, 1000);
  delay(1);
}

void buzzeroff()
{
  noTone(BDPIN_BUZZER);
  delay(1);
}

bool releasebuttoncheck()
{
  val = digitalRead(BDPIN_PUSH_SW_2);
  if (val == HIGH)
  {
    onetimecall = true;
    dxl_wb.goalPosition(dxl_id, (int32_t)0);
    delay(100);
  }
}
