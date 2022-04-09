// This is Board v2\ESP\06_Send_BAT_to_Nano version

// Program Configs ////////////////////////////////////////////////////////////////////////////////////////////

// This is chipid of ESP8266
// Program only run on esp with this chipid
// Chipid canread in debug mode
#define ESP_chipid                              13755325
// green small with reg 5: 961336
// green small with reg 12: 944463
// V1: 13755325
// V2: 13755348

// Debug mode
// ESP send decoded data to its output so serial can monitor by user
// In debug mode ChipID not check and its value send to serial at begin of code
#define debug_mode                              true

// WiFi Station User and Password
#define WiFi_SSID                               "Two_way1"
// !! Password mast be at least 8 chars !!
#define WiFi_password                            "12341235"

// WiFi channel (1-11)
#define WiFi_channel                            5
// WiFi SSID hidden: network name will not show when this option is true
#define WiFi_hidden                             false
// Spcify number of users can connect to ESP access point
#define WiFi_users_limit                        1


// Port and IP for UDP connection. UDP IP is IP_local and PORT is UDP_port
// IP_local, IP_gateway and IP_subnet are related to IPv4 protocol
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

#define SERIAL_FIRST  "1231"
#define SERIAL_SECOND_V "YES"
#define SERIAL_SECOND_I "NO"


// Library Configs ////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ESP8266WiFi.h>    // ESP8266 Library
#include <WiFiUDP.h>        // UDP Protocol Library


// Variables /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
// IP and port of client are required to send data for client


int replyPacketB_cur = 0;
char replyPacketB[255];
char incomingPacket[255];
int packetSize;

// battery variables
int batADC = 0;
int batADC_last = 0;
uint16_t batADC_Avg_index = 0;
float batADC_Avg = 0;
uint32_t batADC_read_time = 0;

char buffer_m[30];

char random_num_m[30];

unsigned char udp_state = 0, serial_state = 0;
unsigned char process = 0;

char udp_reset_str[30];

void setup()
{
  #if debug_mode != true
    if(~ESP.getChipId() != (~ESP_chipid))
      while(true) yield();
  #endif

  
  // Config Serial
  Serial.begin(send_data_baudrate);

  // Config ESP8266 WiFi Station with defined IPs
  bool IPConfigResult = WiFi.softAPConfig(local_IP, gateway, subnet);

  // Set ESP Mode to Access Point (AP)
  WiFi.mode(WIFI_AP);
  // Create WiFi Station
  bool softAPResult = WiFi.softAP(ssid, password, WiFi_channel, WiFi_hidden, WiFi_users_limit);
  
  #if debug_mode == true
    Serial.println("Hello :)");
    
    // Send result of IP config
    Serial.print("Setting soft-AP configuration ... ");
    Serial.println(IPConfigResult ? "Ready" : "Failed!");

    // Swnd result of AP creation
    Serial.print("Setting soft-AP ... ");
    Serial.println(softAPResult ? "Ready" : "Failed!");

    // Send allocated IP to Serial port
    Serial.print("Soft-AP IP address = ");
    Serial.println(WiFi.softAPIP());

    // Send ChipID of ESP
    Serial.printf("Chip id = %d\r\n", ESP.getChipId());
  #endif
  
  // Server Config and Start UDP Server
  Udp.begin(localUdpPort);
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
      //added for test
      //for (int i =0; i < 30; i++) incomingPacket[i] = 0;
      int len = Udp.read(incomingPacket, 20);                                                  // read packet value and save in incomingPacket array
      incomingPacket[len] = 0;                                                                  // define end of string      
      yield();
      switch (udp_state)
      {
        case 0:
        if (strcmp(incomingPacket, UDP_FIRST)  == 0)
        {
            yield();
            Serial.print(UDP_FIRST);
        }
        else
        {
          Udp.beginPacket(remoteUdpIP, remoteUdpPort);
          Udp.write("INVALID FIRST UDP\n");
          Udp.endPacket();          
        }        
        break;

        case 1:
          if (strcmp(incomingPacket, UDP_SECOND)  == 0)
          {
            //strncat(random_num_m, incomingPacket, 20);
            
            yield();
            //Serial.print(UDP_SECOND);
            Serial.print(incomingPacket);
          }
          else if (strcmp(incomingPacket, UDP_FIRST)  == 0)
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
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            Udp.write("INVALID SECOND UDP\n");
            Udp.endPacket(); 
          }
            
        break;

        case 2:
          if (strcmp(incomingPacket, UDP_THIRD)  == 0)
          {
            //strncat(random_num_m, incomingPacket, 20);
            
            yield();
            //Serial.print(UDP_SECOND);
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
          
          else
          {
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            Udp.write("INVALID THIRD UDP\n");
            Udp.endPacket(); 
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
          for(int i = 0; i<20; i++ )
          {
            buffer_m[i] = 0;
          }
          Serial.readBytes(buffer_m, 20);
          //if (strcmp(buffer_m, SERIAL_FIRST)  == 0)
          if(1)
          {
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            //Udp.write("bbbb\n");
            //const char buffer_str[30] = SERIAL_FIRST ;
            
            Udp.write( buffer_m );
            Udp.write("\n");
            Udp.endPacket();
            serial_state = 1;
            udp_state = 1;
          }

        break;

        case 1:
          for(int i = 0; i<20; i++ )
          {
            buffer_m[i] = 0;
          }
          Serial.readBytes(buffer_m, 20);
          if (strcmp(buffer_m, SERIAL_SECOND_V)  == 0)         
          {
            Udp.beginPacket(remoteUdpIP, remoteUdpPort);
            Udp.write("YES\n");
            Udp.endPacket();
            serial_state = 3;
            udp_state = 2;
            //process = 1;
            //udp_state = 0;
          }
          else if (strcmp(buffer_m, SERIAL_SECOND_I)  == 0)
          {
            //string_buffer = String(buffer_m) + "\n";
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
          
        break;

        case 3:
        //process = 1;

        for(int i = 0; i<20; i++ )
          {
            buffer_m[i] = 0;
          }
          Serial.readBytes(buffer_m, 20);
          
          Udp.beginPacket(remoteUdpIP, remoteUdpPort);  
          Udp.write( buffer_m );
          Udp.write("\n");
          Udp.endPacket();
          //Udp.beginPacket(remoteUdpIP, remoteUdpPort);
          //Udp.write("YES\n");
          //Udp.endPacket();
          //serial_state = 3;
          //udp_state = 2;
          process = 1;
          udp_state = 0;
          serial_state = 0;
      
        break;
        
      }
    }   
}

else
{
  
  // Calculate battery average and send to nano
  // Sample rate of ADC is 5K
  if(millis() - batADC_read_time > 10)
  {
    batADC_read_time = millis();
    yield();     // do esp tasks which is realated to WiFi
    batADC_last = batADC;
    batADC = analogRead(A0); 
    yield();     // do esp tasks which is realated to WiFi   
    if(abs(batADC_last - batADC) <= battery_average_sample_accuracy)
    {
      batADC_Avg += batADC;
      batADC_Avg_index += 1;
    }
     
    if(batADC_Avg_index >= battery_average_sample_count)
    {
      yield();     // do esp tasks which is realated to WiFi
      batADC_Avg = batADC_Avg / battery_average_sample_count;
      Serial.println(String((uint32_t)(batADC_Avg * 1000)));
      batADC_Avg = 0;
      batADC_Avg_index = 0;
      yield();     // do esp tasks which is realated to WiFi
    }
  }

  // Check for reciveing UDP package
  packetSize = Udp.parsePacket();
  if (packetSize)   // if package recived
  {
    remoteUdpIP = Udp.remoteIP();             // save sender IP for future works
    remoteUdpPort = Udp.remotePort();         // save sender PORT for future works
    int len = Udp.read(incomingPacket, 255);  // read packet value and save in incomingPacket array
    incomingPacket[len] = 0;                  // define end of string
    //if(strcmp(incomingPacket, UDP_start_command) == 0)
    //{
      program_lock = false;
      // Send answer that program is started
      Udp.beginPacket(remoteUdpIP, remoteUdpPort);                        // Start of UDP packet
      Udp.write("WIFI OK\r\n");                                // Write UDP message
      //Serial.print(UDP_start_command);
      #if debug_mode == true                                              // Send ESP chip id
        ("My Chipid is "+ String(ESP.getChipId()) +"\r\n").toCharArray(replyPacketB, 100);
        //Udp.write(replyPacketB);     // Write UDP message
      #endif
      Udp.endPacket();                                                    // End of UDP packet

      // wait until new line receive from UART
      incomingPacket[0] == '\0';
      while(incomingPacket[0] != '\n')
      {
        while(Serial.available() < 1) yield();
        Serial.readBytes(incomingPacket, 1);
        #if decode_data == true
          incomingPacket[0] = (char)(decode_formula(((int)(incomingPacket[0]))));
        #endif
        yield();     // do esp tasks which is realated to WiFi
      }

      replyPacketB_cur = 0;

      
    //}
  }


  // Don't recive from uart and dont send to UDP if program is lock
  if(program_lock) return;
  
  // Check if data is ready from UART and save result in replyPacketB array
  while ((Serial.available() > 0) && (replyPacketB_cur < 254)) {
    yield();    // do esp tasks which is realated to WiFi
    Serial.readBytes(incomingPacket, 1);
    #if decode_data == true
      incomingPacket[0] = (char)(decode_formula(((int)(incomingPacket[0]))));
    #endif
    if(incomingPacket[0] == '\1') continue;                   // To ignore SOB byte
    replyPacketB[replyPacketB_cur] = incomingPacket[0];   
    replyPacketB_cur++;
    if(incomingPacket[0] == '\n') break;
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
