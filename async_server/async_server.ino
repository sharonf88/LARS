//#include "WiFi.h"
//#include "AsyncUDP.h"
#include <ESP8266WiFi.h>
#include "ESPAsyncUDP.h"
#include "WiFiUdp.h"
//#include <String>
const char *my_ssid = "esp8266_server";
const char *password = "12345678";

int port = 1234;

AsyncUDP recv_udp;
WiFiUDP send_udp;
int packet_num = 0;
String packet_num_str;
//const char * msg = "test\n";
 char charBuf[50];

void setup(){
    Serial.begin(115200);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(my_ssid, password);
    delay(100);

      if(recv_udp.listen(port)) {
        recv_udp.onPacket([](AsyncUDPPacket packet) {
            Serial.print("Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();  
          });
      }
}

void loop() {
    delay(1000);
    
      send_udp.beginPacket(IPAddress(192,168,4,255), 1234);
      send_udp.write("test ");
      

      packet_num_str = String(packet_num);
      packet_num_str.toCharArray(charBuf, 50);
      send_udp.write(charBuf);
//      send_udp.write(packet_num);
      send_udp.write("\n");
      send_udp.endPacket();
      Serial.println("Sended multicast call from ESP-1");
      packet_num++;
}
