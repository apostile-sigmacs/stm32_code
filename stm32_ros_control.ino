/*
 * PROJECT TITLE: STM32 100KG PAYLOAD CODE
 * DATE MODIIFED:
 */

#define USE_STM32_HW_SERIAL
#define USE_MPU9250_IMU

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <Imu.h>
#include <apostle_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <Adafruit_NeoPixel.h>

#define WHEEL_RADIUS 0.062 //in m
#define ROBOT_WIDTH 0.5 // in m
#define NUMPIXELS 4

#define BR1 PB12
#define BR2 PB13
#define PIXEL_LED PB14
#define LED PB15

float R1 = 12000.00;
float R2 = 2200.00;

#define IMU_PUBLISH_RATE 20 //hz
using namespace ros;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace geometry_msgs;

float Velocity_Left, Velocity_Right;

NodeHandle nh;
apostle_msgs::Imu raw_imu_msg;
Float32 voltage;

Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
Publisher voltage_pub("voltage", &voltage);

Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_LED, NEO_GRB + NEO_KHZ800);

void cmd_vel_callback(const Twist& cmd_msg)
{
  Velocity_Left = (cmd_msg.linear.x - cmd_msg.angular.z * (ROBOT_WIDTH / 2)) / (2 * PI * WHEEL_RADIUS) ;
  Velocity_Right = (cmd_msg.linear.x + cmd_msg.angular.z * (ROBOT_WIDTH / 2)) / (2 * PI * WHEEL_RADIUS);


  Velocity_Left = Velocity_Left * 60;
  Velocity_Right = Velocity_Right * 60;

  if ((Velocity_Left == 0) && (Velocity_Right == 0))red_glow();
  else if (((Velocity_Left < 0) && (Velocity_Right > 0)) || (Velocity_Left < Velocity_Right))
  {
    amber_glow(0, 1);
    angular_green_glow(2, 3);
  }
  else if (((Velocity_Left > 0) && (Velocity_Right < 0)) || (Velocity_Left > Velocity_Right))
  {
    amber_glow(2, 3);
    angular_green_glow(0, 1);
  }
  else green_glow();
}

Subscriber<Twist>cmd_sub("cmd_vel", cmd_vel_callback);

void setup() {

  pixels.begin();
  pixels.clear();

  nh.initNode();
  nh.getHardware();
  nh.advertise(raw_imu_pub);
  nh.advertise(voltage_pub);
  nh.subscribe(cmd_sub);

  while (!nh.connected())
  {
    nh.spinOnce();
  }

  nh.loginfo("STM32 INITIALIZED...");
}

void loop() {
  static unsigned long prev_imu_time = 0, prev_time;
  static bool imu_is_initialized = false;

  if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    //sanity check if the IMU is connected
    if (!imu_is_initialized)
    {
      imu_is_initialized = initIMU();
      imu_is_initialized ? nh.loginfo("IMU Initialized") : nh.logfatal("IMU failed to initialize. Check your IMU connection.");
    }
    if (imu_is_initialized)
    {
      publishIMU();
    }
    prev_imu_time = millis();
  }

  nh.spinOnce();
  if (!nh.connected())red_glow();
}


void publishIMU()
{
  int temp = (analogRead(PA0) * 3.3) / 1024.0;//

  voltage.data = (temp / (R2 / (R1 + R2) )) / 0.99; 
  
  //pass accelerometer data to imu object
  raw_imu_msg.linear_acceleration = readAccelerometer();

  //pass gyroscope data to imu object
  raw_imu_msg.angular_velocity = readGyroscope();

  //pass accelerometer data to imu object
  raw_imu_msg.magnetic_field = readMagnetometer();

  //publish raw_imu_msg
  raw_imu_pub.publish(&raw_imu_msg);
  voltage_pub.publish(&voltage);
}

void amber_glow(int led_position1, int led_position2)
{
  pixels.setPixelColor(led_position1, pixels.Color(0xFF, 0XBF, 0x00));
  pixels.setPixelColor(led_position2, pixels.Color(0xFF, 0XBF, 0x00));
  pixels.show();
}

void angular_green_glow(int led1pos, int led2pos)
{
  pixels.setPixelColor(led1pos, pixels.Color(0, 0XFF, 0x00));
  pixels.setPixelColor(led2pos, pixels.Color(0, 0XFF, 0x00));
  pixels.show();
}
void green_glow()
{
  for (int i = 0; i < NUMPIXELS; i++)
    pixels.setPixelColor(i, pixels.Color(0, 255, 0));
  pixels.show();
}

void red_glow()
{
  for (int i = 0; i < NUMPIXELS; i++)
    pixels.setPixelColor(i, pixels.Color(255, 0, 0));
  pixels.show();
}
