#include <Wire.h>
#include "MS5837.h"
#include <ESP8266WiFi.h>
#include "ESPAsyncUDP.h"
#include "WiFiUdp.h"

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"
// See limitations of Arduino SoftwareSerial
// SoftwareSerial serial(10,11);  
// Create the RoboClaw object, passing the pointer to the hardware serial object
// and the serial timeout value
RoboClaw roboclaw = RoboClaw(&Serial1, 10000);  //Hardware serial 1 check pins
#define address 0x80

const char *my_ssid = "esp8266_server";
const char *password = "12345678";

int port = 1234;

AsyncUDP recv_udp;
WiFiUDP send_udp;
int packet_num = 0;
String packet_num_str;
char charBuf[50];


MS5837 sensor;

float tmp;
double depth      =   0 ;        // Sine input
double depth_p    =   0 ;       // Sensor depth in meters
float setpoint    =   0 ;      // Req. depth to maintain in meters
double pid_scalar =   0 ;
double PIDout     =  0.0;

double P = 0.0;
double I = 0.0;
double D = 0.0;

double kp = 500 ;
double ki = 10;
double kd = 50;
double ka = 10; //for anti-windup

double dSat = 0.0 ; 

double  PrevTime   = 0.0;
double  PrevErr    = 0.0;
double  CurTime    = 0.0;
double  deltaT     = 0.0;
double  err        = 0.0;
double  dErr       = 0.0;

int leakPin2 = 2;    // Leak Signal Pin 1
int leakPin3 = 3;    // Leak Signal Pin 2
int leak2 = 0;      // 0 = Dry , 1 = Leak
int leak3 = 0;      // 0 = Dry , 1 = Leak
int flag = 0;       // 0 = P.sensor (Sea), 1 = Sine (Simulation)

bool run=true;

void setup() {
  // Leak //
//  pinMode(leakPin2, INPUT);
//  pinMode(leakPin3, INPUT);
  // initialize serial, use the same boudrate in the Simulink Config block
  Serial.begin(115200);
  // for hardware - motor
  Serial1.begin(115200); 
  // Motor Driver
  roboclaw.begin(38400);
 // Pressure sensor
  Wire.begin();
  
  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar10: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(50);}
    
  // WIFI
    WiFi.mode(WIFI_AP);
    WiFi.softAP(my_ssid, password);
    delay(100);
    
      if(recv_udp.listen(port)) {
        recv_udp.onPacket([](AsyncUDPPacket packet) {
            Serial.print("Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
             
            // Conditions
            // RUN & STOP MOTOR  
            if (strncmp((char*)(packet.data()), "stop", 4) == 0){
               Serial.print("I stopped ");
               Serial.println(); 
               run=false;
            }
            if (strncmp((char*)(packet.data()), "run", 3) == 0){
               Serial.print("Running again ");
               Serial.println(); 
               run=true;
            }

            // Kp Ki Kd Ka Input
            if (strncmp((char*)(packet.data()), "kp", 2) == 0){
               Serial.print("kp: ");
               kp = String((char*)(packet.data())).substring(2).toFloat();
               Serial.println(kp); 
            }  
            if (strncmp((char*)(packet.data()), "ki", 2) == 0){
               Serial.print("ki: ");
               ki = String((char*)(packet.data())).substring(2).toFloat();
               Serial.println(ki); 
            }
            if (strncmp((char*)(packet.data()), "kd", 2) == 0){
               Serial.print("kd: ");
               kd = String((char*)(packet.data())).substring(2).toFloat();
               Serial.println(kd); 
            }   
             if (strncmp((char*)(packet.data()), "ka", 2) == 0){
               Serial.print("ka: ");
               ka = String((char*)(packet.data())).substring(2).toFloat();
               Serial.println(ka); 
            }                          
            // Input Sine Wave
            if (strncmp((char*)(packet.data()), "sine", 4) == 0){
               Serial.print("Sine Wave: ");
               depth = String((char*)(packet.data())).substring(4).toFloat();
               Serial.println(depth);
               flag=1; 
            }      
            
            // Input Pressure Sensor
            if (strncmp((char*)(packet.data()), "sensor", 6) == 0){
               Serial.print("Pressure Sensor Read Depth: ");
               Serial.println(depth_p); 
               flag=0;
            }  
                        
           // Input Depth Setpoint
            if (strncmp((char*)(packet.data()), "depth", 5) == 0){
               Serial.print("Input Depth: ");
               setpoint = String((char*)(packet.data())).substring(5).toFloat();
               Serial.println(setpoint); 
            }          
       });
   }


  // Dpeth sensor setup   
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(1029); // kg/m^3 (997 freshwater, 1029 for seawater)
}

void loop(){
  
  // Update pressure readings
  sensor.read();
  depth_p = sensor.depth();
  if (flag == 0){
    depth = depth_p;}
  
 // Handle depth Boundaries at sea
  //  if (depth_p < 0.2 || depth_p > 6){
 //    roboclaw.ForwardM1(address,0);
//  }
  
/// Error reading handle //
    if (depth > 100 || depth<-100){
        depth = 0;
    //Serial.println("Depth sensor error");
    }


  // PID
  err =  (depth - setpoint);

  CurTime  = millis(); // seconds conv.
  deltaT   = (CurTime - PrevTime) / 1000;
  PrevTime = CurTime            ;

  dErr     = err - PrevErr      ;
  PrevErr  = err                ;
  
  P = err * kp;

 I+=  ((err * ki) + (ka  * dSat)) * deltaT  ; 
  Serial.print("I: ");
  Serial.print(I); 

  if (deltaT * kd != 0){  
   D = (dErr * kd) / deltaT;}
   Serial.print("     D: ");
   Serial.println(D);
 
if (flag == 0){
  PIDout = P + D + I ;}
  else{
  PIDout = P + D ;}


  if (PIDout > 127){
      pid_scalar = 127  ; 
  }

  else if (PIDout < -127){
      pid_scalar = -127 ;
  }

  else {
      pid_scalar =   PIDout;
    }

  dSat      = pid_scalar - PIDout ; // for Anti windup

  if (run == false){
    roboclaw.ForwardM1(address,0); 
  }

  else {
   if (pid_scalar < 0){
    pid_scalar = pid_scalar * -1;
   // Serial.println("down");
   roboclaw.ForwardM1(address,pid_scalar);  //start Motor1 backward
   delay(10);
   }

   else if (pid_scalar > 0) {
    // Serial.println("up");
   roboclaw.BackwardM1(address,pid_scalar);  //start Motor1 forward
    delay(10);
   }
 }
  
 // WIFI //
 
      send_udp.beginPacket(IPAddress(192,168,4,255), 1234);
      send_udp.write("depth ");
      packet_num_str = String(depth_p);
      packet_num_str.toCharArray(charBuf, 50);
      send_udp.write(charBuf);
      // Motor On or Off
      send_udp.write(" running ");
      packet_num_str = String(run);
      packet_num_str.toCharArray(charBuf, 50);
      send_udp.write(charBuf);
            
      send_udp.write("\n");
      send_udp.endPacket();
      packet_num++;
}
