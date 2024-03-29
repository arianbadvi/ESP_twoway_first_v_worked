// Program Configs ////////////////////////////////////////////////////////////////////////////////////////////
#define ESP_chipid                              13755325
// Debug mode
// ESP send decoded data to its output so serial can monitor by user
// In debug mode ChipID not check and its value send to serial at begin of code
#define debug_mode                              true
// WiFi Station User and Password
#define WiFi_SSID                               "two_way10"
// !! Password mast be at least 8 chars !!
#define WiFi_password                            "12341235"
// WiFi channel (1-11)
#define WiFi_channel                            5
// WiFi SSID hidden: network name will not be shown when this option is true
#define WiFi_hidden                             false
// Spcify number of users can be connected  to the ESP access point
#define WiFi_users_limit                        1
#define IP_local                                192,168,4,2
#define IP_gateway                              192,168,4,1
#define IP_subnet                               255,255,255,0          
#define UDP_port                                4210
// UDP start command: Computer must send this command to ESP in case of start working
#define UDP_start_command                       "s"
// battery number of average samples (each sample increase update wait of battery for 10ms)
#define battery_average_sample_count            50
// ADC sample accuracy. if diffrence between two sample is more than this number, the last one will remove (2 - 20)
#define battery_average_sample_accuracy         2
// Baudrate of sending data
// in ESP this option must be the same
#define send_data_baudrate                      115200
// decode data that recived from uart
// Prevent from stealing data
// in Arduino this option must be the same
#define decode_data                             false
// in Arduino this option must be reverse
// formula to encode each char: x is char (between 0 and 255)
#define decode_formula(x)                       (255 - x)
#define UDP_FIRST   "aaaaaaaa"
#define UDP_SECOND  "24682468246824682468"
#define UDP_THIRD   "11110011"
#define STATUS_UDP  "11111111"
#define MAIN_BOARD_SEND_STATUS_STRING "ssssssss"
#define SERIAL_FIRST  "1231"
#define SERIAL_SECOND_V "YES"
#define SERIAL_SECOND_I "NO"

#include <ESP8266WiFi.h>    // ESP8266 Library
#include <WiFiUDP.h>        // UDP Protocol Library

// if this variable is one ESP will not send any data
bool program_lock = true;
// WiFi Station User and Password
const char *ssid = WiFi_SSID;
const char *password = WiFi_password; 
// IPv4 variables
IPAddress local_IP(IP_local);
IPAddress gateway(IP_gateway);
IPAddress subnet(IP_subnet);
// UDP connection variables
WiFiUDP Udp;                            // Create and object of UDP Class
unsigned int localUdpPort = UDP_port;   // local port to listen on (On computer side remote port is UDP_port)
IPAddress remoteUdpIP;                  // IP of client (Here means computer that sent request to ESP)
unsigned int remoteUdpPort;             // Port of client
int replyPacketB_cur = 0;
char replyPacketB[255];
char incomingPacket[255];
int packetSize;
char buffer_m[30];
char random_num_m[30];
unsigned char udp_state = 0, serial_state = 0;
unsigned char process = 0;
char udp_reset_str[30];
char str[30];

void(* resetFunc) (void) = 0;

void setup()
{
  #if debug_mode != true
    if(~ESP.getChipId() != (~ESP_chipid))
      while(true) yield();
  #endif
  Serial.begin(send_data_baudrate);
  bool IPConfigResult = WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.mode(WIFI_AP);
  bool softAPResult = WiFi.softAP(ssid, password, WiFi_channel, WiFi_hidden, WiFi_users_limit);
  
  #if debug_mode == true
    Serial.println("Hello :)");
    Serial.print("Setting soft-AP configuration ... ");
    Serial.println(IPConfigResult ? "Ready" : "Failed!");
    Serial.print("Setting soft-AP ... ");
    Serial.println(softAPResult ? "Ready" : "Failed!");
    Serial.print("Soft-AP IP address = ");
    Serial.println(WiFi.softAPIP());
    Serial.printf("Chip id = %d\r\n", ESP.getChipId());
  #endif
  Udp.begin(localUdpPort);
  Serial.printf("Chip id = %d\r\n", ESP.getChipId());
}

void loop()
{
  if(process == 0)
  {
    yield();
    packetSize = Udp.parsePacket();                                                               // Check for reciveing UDP package  
    if (packetSize)                                                                               // if package recived
    {      
      remoteUdpIP = Udp.remoteIP();                                                             // save sender IP for future works
      remoteUdpPort = Udp.remotePort();                                                         // save sender PORT for future works
      int len = Udp.read(incomingPacket, 20);                                                  // read packet value and save in incomingPacket array
      incomingPacket[len] = 0;                                                                  // define end of string      
      yield();
      switch (udp_state)
      {
        case 0:
        {
          if (strcmp(incomingPacket, UDP_FIRST)  == 0)
          {
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            sprintf(str, "Chip id = %d\r\n", ESP.getChipId());
            Udp.write((char *)(str));
            Udp.endPacket(); 
            yield();
            Serial.print(UDP_FIRST);
          }
          else
          {
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            Udp.write("INVALID FIRST UDP\n");
            Udp.endPacket();          
          }  
        }      
        break;
        case 1:
        {
          if (strcmp(incomingPacket, UDP_FIRST)  == 0)
          {
            yield();        
            for (int i = 0; i < 30; i++)  udp_reset_str[i] = 0;
            for (int i = 0; i < strlen(UDP_SECOND); i++) udp_reset_str[i] = 'R';                     
            Serial.print(udp_reset_str);          
            udp_state = 0;
            serial_state = 0;
          }
          else
          {
            Serial.print(incomingPacket);
          }
        }
        break;
        case 2:
        {
          if (strcmp(incomingPacket, UDP_THIRD)  == 0)
          {         
            yield();
            Serial.print(UDP_THIRD);           
          }
          else if (strcmp(incomingPacket, UDP_FIRST)  == 0)
          {
            yield();
            for (int i = 0; i < 30; i++)  udp_reset_str[i] = 0;
            for (int i = 0; i < strlen(UDP_THIRD); i++) udp_reset_str[i] = 'R';          
            Serial.print(udp_reset_str);
            udp_state = 0;
            serial_state = 0;
          }     
          else if(strcmp(incomingPacket, STATUS_UDP) == 0)
          {
            yield();
            serial_state = 4;
            Serial.print(MAIN_BOARD_SEND_STATUS_STRING);
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            Udp.write("Sent to micro\n");
            Udp.endPacket(); 
          }                 
          else
          {
            yield();
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            Udp.write("INVALID THIRD UDP\n");
            Udp.endPacket(); 
          }
        }   
        break;
      }      
    }  
    if(Serial.available())
    {
      yield();
      switch (serial_state)
      {
        case 0:
        {
          for(int i = 0; i<20; i++ )  buffer_m[i] = 0;
          Serial.readBytes(buffer_m, 20);
          if (strcmp(buffer_m, "SERIAL")  == 0)
          {
            Serial.print("OK");
            WiFiMode(WIFI_STA);
            WiFi.disconnect(); 
            WiFi.mode(WIFI_OFF);
            delay(1000);                            
          }
          yield();
          Udp.beginPacket(remoteUdpIP, remoteUdpPort); 
          Udp.write( buffer_m );
          Udp.write("\n");
          Udp.endPacket();
          serial_state = 1;
          udp_state = 1;
        }
        break;
        case 1:
        {
          yield();
          for(int i = 0; i<20; i++ )  buffer_m[i] = 0;
          Serial.readBytes(buffer_m, 20);
          if (strcmp(buffer_m, SERIAL_SECOND_V)  == 0)         
          {
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            Udp.write("YES\n");
            Udp.endPacket();
            serial_state = 3;
            udp_state = 2;
          }
          else if (strcmp(buffer_m, SERIAL_SECOND_I)  == 0)
          {
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            Udp.write("NO\n");
            Udp.endPacket();
            serial_state = 0;
            udp_state = 0;            
          }
          else
          {
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            Udp.write("Connection problem\n");
            Udp.endPacket();
            serial_state = 0;
            udp_state = 0;
          }
        }
        break;
        case 3:
        {
          yield();
          for(int i = 0; i<20; i++ )  buffer_m[i] = 0;
          Serial.readBytes(buffer_m, 20);      
          Udp.beginPacket(remoteUdpIP, remoteUdpPort);  
          Udp.write( buffer_m );
          Udp.write("\n");
          Udp.endPacket();
          process = 1;
          udp_state = 0;
          serial_state = 0;
        }
        break;
        case 4:
        {
          for(int i = 0; i<20; i++ )  buffer_m[i] = 0;
          Serial.readBytes(buffer_m, 9);      
          Udp.beginPacket(remoteUdpIP, remoteUdpPort);  
          Udp.write(buffer_m);
          Udp.write("\n");
          Udp.endPacket();
          serial_state = 3;
        }
        break;
      }
    }   
  }
  else if (process == 1)
  {
    yield();
    // Check for reciveing UDP package
    packetSize = Udp.parsePacket();
    if (packetSize)                             // if package has been recived
    {
      remoteUdpIP = Udp.remoteIP();             // save sender IP for future works
      remoteUdpPort = Udp.remotePort();         // save sender PORT for future works
      int len = Udp.read(incomingPacket, 255);  // read packet value and save in incomingPacket array
      incomingPacket[len] = 0;                  // define end of string
      //added for test
      if ( strcmp(incomingPacket, UDP_FIRST) == 0 )
      {
        Serial.print('R');
        process = 0;
        udp_state = 0;
        serial_state = 0;
        resetFunc();   
      }
        program_lock = false;
        #if debug_mode == true
          ("My Chipid is "+ String(ESP.getChipId()) +"\r\n").toCharArray(replyPacketB, 100);
        #endif
        incomingPacket[0] == '\0';
        while(incomingPacket[0] != '\n')
        {
          while(Serial.available() < 1) yield();
          Serial.readBytes(incomingPacket, 1);
          #if decode_data == true
            incomingPacket[0] = (char)(decode_formula(((int)(incomingPacket[0]))));
          #endif
          yield();     // do esp tasks which is realated to WiFi
          if (process == 0) break;
        }
        replyPacketB_cur = 0;
      }
    // Don't recive from uart and dont send to UDP if program is lock
    if(program_lock) return;
    // Check if data is ready from UART and save result in replyPacketB array
    while ((Serial.available() > 0) && (replyPacketB_cur < 254)) 
    {
      yield();    // do esp tasks which is realated to WiFi
      Serial.readBytes(incomingPacket, 1);
      #if decode_data == true
        incomingPacket[0] = (char)(decode_formula(((int)(incomingPacket[0]))));
      #endif
      if(incomingPacket[0] == '\1') continue;                   // To ignore SOB byte
      replyPacketB[replyPacketB_cur] = incomingPacket[0];   
      replyPacketB_cur++;
      if(incomingPacket[0] == '\n') break;
      if (process == 0) break;
    }
    // Sent replyPacketB to WiFi
    if((replyPacketB_cur >= 254) || ((replyPacketB_cur > 0) && (incomingPacket[0] == '\n')))
    {
      yield();    // do esp tasks which is realated to WiFi
      replyPacketB[replyPacketB_cur++] = 0;
      Udp.beginPacket(remoteUdpIP, remoteUdpPort);
      Udp.write(replyPacketB);
      Udp.endPacket();
      #if debug_mode == true
        // Serial.print(replyPacketB);
      #endif
      replyPacketB_cur = 0;
    }  
  }
}
