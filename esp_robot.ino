/*
  WiFiTelnetToSerial - Example Transparent UART to Telnet Server for esp8266

  Copyright (c) 2020 Vasanthakumar M. 

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  email: vasanthbe@gmail.com
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>

#include <algorithm> // std::min
#include <stdlib.h>

#ifndef STASSID
#define STASSID "NodeMCU"
#define STAPSK  "robot@1234"
#endif

////////////////////////////////////////////////////////////

#if SERIAL_LOOPBACK
#undef BAUD_SERIAL
#define BAUD_SERIAL 3000000
#include <esp8266_peri.h>
#endif

#if SWAP_PINS
#include <SoftwareSerial.h>
SoftwareSerial* logger = nullptr;
#else
#define logger (&Serial)
#endif

#define STACK_PROTECTOR  512 // bytes

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 2
const char* ssid = STASSID;
const char* password = STAPSK;

const int port = 23;

WiFiServer server(port);
WiFiClient serverClients[MAX_SRV_CLIENTS];

#define MIN_BUFF_LEN 2
#define FREE 0
#define BUSY 1
#define READ 3
#define WRITE 4

//Encoder interrupt pins left D7 right D5
uint8_t interruptPin1 = D7;
volatile byte interruptCounter1 = 0;
uint8_t interruptPin2 = D5;
volatile byte interruptCounter2 = 0;

#define PWMA  5
#define PWMB  4 
#define DA    0 
#define DB    2 

#define PWM_DUTY_MIN 0
#define PWM_DUTY_MAX 1024

unsigned char PROCESS_FLAG;
unsigned char OUT_BUFFER;
unsigned char IN_BUFFER;

char sdata[100];
int slen;

char esp_control;
int esp_duty_ma;
int esp_duty_mb;
char esp_dir_mab;

int ProcessData(uint8_t*,size_t);
uint8_t Phrase_Sensor(uint8_t *data, uint8_t index);
uint8_t Phrase_Control(uint8_t *data, uint8_t index);
uint8_t Phrase_Motor_DIR_Data(uint8_t *data, uint8_t index);
uint8_t Phrase_Motor_Duty_Data(uint8_t *data, uint8_t index);
uint8_t Phrase_Encoder(uint8_t *data, uint8_t index);

int ProcessData(uint8_t *data,size_t len)
{
  uint8_t index = 0;
  if(len > MIN_BUFF_LEN)
  {
    if((data[0]==':')&&(data[1]=='#'))
    {
      index = 2;
      switch(data[index])
      {
        case 'A':
          index = Phrase_Control(data,++index);
          break;
        case 'B':
          index = Phrase_Motor_DIR_Data(data,++index);
          break;
        case 'C':
          index = Phrase_Motor_Duty_Data(data,++index);
          break;
        case 'D':
          index = Phrase_Sensor(data,++index);
          break;
        case 'E':
          index = Phrase_Control(data,++index);
          index = Phrase_Motor_DIR_Data(data,index);
          index = Phrase_Motor_Duty_Data(data,index);
          break;
        case 'F':
          index = Phrase_Encoder(data,++index);
          break;
        default:
          sdata[0] = ':';
          sdata[1] = '#';
          sdata[2] = 'E';
          sdata[3] = 'R';
          sdata[4] = 'R';
          sdata[5] = ' ';
          sdata[6] = 'D';
          sdata[7] = 'A';
          sdata[8] = 'T';
          sdata[9] = 'A';
          slen = 10;
          Serial.print(":#ERR DATA RANGE\r\n");
          break;
      }
    }
  }
  else
  {
    sdata[0] = ':';
    sdata[1] = '#';
    sdata[2] = 'E';
    sdata[3] = 'R';
    sdata[4] = 'R';
    sdata[5] = ' ';
    sdata[6] = 'F';
    sdata[7] = 'O';
    sdata[8] = 'R';
    sdata[9] = 'M';
    sdata[10] = 'A';
    sdata[11] = 'T';
    slen = 12;
    Serial.printf(":#ERR FORMAT %i\r\n", len);
  }
  return 0;  
}

uint8_t Phrase_Sensor(uint8_t *data, uint8_t index)
{
  uint8_t i = 0;
  
  sdata[i++] = ':';
  sdata[i++] = '#';
  
  switch(data[index]){
      case '0':
        sdata[i++] = 'A';
        sdata[i++] = (esp_control);
        //Serial.print("got 0 index\r\n");
        break;
      case '1':
        sdata[i++] = 'A';
        sdata[i++] = (esp_control);
        sdata[i++] = 'B';
        sdata[i++] = (esp_dir_mab);
        //Serial.print("got 1 index\r\n");
        break;
      case '2':
        sdata[i++] = 'A';
        sdata[i++] = esp_control;
        sdata[i++] = 'B';
        sdata[i++] = esp_dir_mab;
        sdata[i++] = 'C';
        sdata[i++] = (esp_duty_ma >> 7);
        sdata[i++] = (esp_duty_ma & 0xFF);
        sdata[i++] = (esp_duty_mb >> 7);
        sdata[i++] = (esp_duty_mb & 0xFF);
        break;       
      case '3':
        sdata[i++] = 'A';
        sdata[i++] = esp_control;
        sdata[i++] = 'B';
        sdata[i++] = esp_dir_mab;
        sdata[i++] = 'C';
        sdata[i++] = (esp_duty_ma >> 7);
        sdata[i++] = (esp_duty_ma & 0xFF);
        sdata[i++] = (esp_duty_mb >> 7);
        sdata[i++] = (esp_duty_mb & 0xFF);
        sdata[i++] = 'D';
        sdata[i++] = (interruptCounter1 >> 7);
        sdata[i++] = (interruptCounter1 & 0xFF);
        sdata[i++] = (interruptCounter2 >> 7);
        sdata[i++] = (interruptCounter2 & 0xFF);
        interruptCounter1 = 0;
        interruptCounter2 = 0;
        break;
      default:
        sdata[i++] = '0';
        break;
        sdata[i++] = '\r';
        sdata[i++] = '\n';
  }
  //Serial.println(sdata);
  slen = i;
}

uint8_t Phrase_Control(uint8_t *data, uint8_t index)
{
  esp_control = data[index];
  return(++index);
}

uint8_t Phrase_Motor_DIR_Data(uint8_t *data, uint8_t index)
{
    esp_dir_mab = data[index];
    
    switch(esp_dir_mab){
      case '1':
        digitalWrite(DA, LOW);
        digitalWrite(DB, LOW);
        break;
      case '2':
        digitalWrite(DA, HIGH);
        digitalWrite(DB, HIGH);  
        break;
      case '3':
        digitalWrite(DA, LOW);
        digitalWrite(DB, HIGH);
        break;
      case '4':
        digitalWrite(DA, HIGH);
        digitalWrite(DB, LOW);  
        break;
      }
      
    return(++index);
}

uint8_t Phrase_Motor_Duty_Data(uint8_t *data, uint8_t index)
{
    esp_duty_ma = (data[index] << 7)|data[index + 1];
    esp_duty_mb = (data[index + 2] << 7)|data[index + 3];
  
    if(esp_duty_ma < PWM_DUTY_MIN)  esp_duty_ma = PWM_DUTY_MIN;
    else if(esp_duty_ma > PWM_DUTY_MAX)  esp_duty_ma = PWM_DUTY_MAX;
    analogWrite(PWMA,esp_duty_ma);

    if(esp_duty_mb < PWM_DUTY_MIN)  esp_duty_mb = PWM_DUTY_MIN;
    else if(esp_duty_mb > PWM_DUTY_MAX)  esp_duty_mb = PWM_DUTY_MAX;
    analogWrite(PWMB,esp_duty_mb);
    
    return(index+4);
}

uint8_t Phrase_Encoder(uint8_t *data, uint8_t index)
{
    interruptCounter1 = (data[index] << 7)|data[index + 1];
    interruptCounter2 = (data[index + 2] << 7)|data[index + 3];
    return(index+4);
}

/*
    SWAP_PINS:
   0: use Serial1 for logging (legacy example)
   1: configure Hardware Serial port on RX:GPIO13 TX:GPIO15
      and use SoftwareSerial for logging on
      standard Serial pins RX:GPIO3 and TX:GPIO1
*/

//#define SWAP_PINS 0

/*
    SERIAL_LOOPBACK
    0: normal serial operations
    1: RX-TX are internally connected (loopback)
*/

#define SERIAL_LOOPBACK 0

#define BAUD_SERIAL 115200
#define BAUD_LOGGER 115200
#define RXBUFFERSIZE 1024

void ICACHE_RAM_ATTR handleInterrupt1() {
  interruptCounter1++;
}

void ICACHE_RAM_ATTR handleInterrupt2() {
  interruptCounter2++;
}

void setup() {
  PROCESS_FLAG = FREE;
  IN_BUFFER = 0;
  OUT_BUFFER = 0;
  pinMode(PWMA, OUTPUT); 
  pinMode(PWMB, OUTPUT); 
  pinMode(DA, OUTPUT); 
  pinMode(DB, OUTPUT);

  esp_control=0;
  esp_dir_mab=0;
  esp_duty_ma=0;
  esp_duty_mb=0;

    //configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(interruptPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), handleInterrupt1, RISING);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), handleInterrupt2, RISING);
  
  Serial.begin(BAUD_SERIAL);
  Serial.setRxBufferSize(RXBUFFERSIZE);

#if SWAP_PINS
  Serial.swap();
  // Hardware serial is now on RX:GPIO13 TX:GPIO15
  // use SoftwareSerial on regular RX(3)/TX(1) for logging
  logger = new SoftwareSerial(3, 1);
  Serial.begin(BAUD_LOGGER);
  Serial.enableIntTx(false);
  Serial.println("\n\nUsing SoftwareSerial for logging");
#else
  Serial.begin(BAUD_LOGGER);
  Serial.println("\n\nUsing Serial1 for logging");
#endif
  Serial.println(ESP.getFullVersion());
  Serial.printf("Serial baud: %d (8n1: %d KB/s)\n", BAUD_SERIAL, BAUD_SERIAL * 8 / 10 / 1024);
  Serial.printf("Serial receive buffer size: %d bytes\n", RXBUFFERSIZE);

#if SERIAL_LOOPBACK
  USC0(0) |= (1 << UCLBE); // incomplete HardwareSerial API
  Serial.println("Serial Internal Loopback enabled");
#endif

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();
  Serial.print("connected, address=");
  Serial.println(WiFi.localIP());

  //start server
  server.begin();
  server.setNoDelay(true);
  //Clearing Interupt counts - encoder counts
  interruptCounter1 = 0;
  interruptCounter2 = 0;

  Serial.print("Ready! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.printf(" %d' to connect\n", port);
}

void loop() {
  //check if there are any new clients
  if (server.hasClient()) {
    //find free/disconnected spot
    int i;
    for (i = 0; i < MAX_SRV_CLIENTS; i++)
      if (!serverClients[i]) { // equivalent to !serverClients[i].connected()
        serverClients[i] = server.available();
        Serial.print("New client: index ");
        Serial.print(i);
        break;
      }

    //no free/disconnected spot so reject
    if (i == MAX_SRV_CLIENTS) {
      server.available().println("busy");
      // hints: server.available() is a WiFiClient with short-term scope
      // when out of scope, a WiFiClient will
      // - flush() - all data will be sent
      // - stop() - automatically too
      Serial.printf("server is busy with %d active connections\n", MAX_SRV_CLIENTS);
    }
  }

  //check TCP clients for data
#if 1
  // Incredibly, this code is faster than the bufferred one below - #4620 is needed
  // loopback/3000000baud average 348KB/s
  for (int i = 0; i < MAX_SRV_CLIENTS; i++)
    while (serverClients[i].available() && Serial.availableForWrite() > 0) {
      // working char by char is not very efficient
      //Serial.write(serverClients[i].read());
      size_t maxToSerial = std::min(serverClients[i].available(), Serial.availableForWrite());
      maxToSerial = std::min(maxToSerial, (size_t)STACK_PROTECTOR);
      uint8_t buf[maxToSerial];
      size_t tcp_got = serverClients[i].read(buf, maxToSerial);
      //size_t serial_sent = Serial.write(buf, tcp_got);
      //Need to process the data here
      ProcessData(buf, maxToSerial);
      
      //Clearing the data
      memset(buf,0,tcp_got);
    }
#else
  // loopback/3000000baud average: 312KB/s
  for (int i = 0; i < MAX_SRV_CLIENTS; i++)
    while (serverClients[i].available() && Serial.availableForWrite() > 0) {
      size_t maxToSerial = std::min(serverClients[i].available(), Serial.availableForWrite());
      maxToSerial = std::min(maxToSerial, (size_t)STACK_PROTECTOR);
      uint8_t buf[maxToSerial];
      size_t tcp_got = serverClients[i].read(buf, maxToSerial);
      size_t serial_sent = Serial.write(buf, tcp_got);
      if (serial_sent != maxToSerial) {
        Serial.printf("len mismatch: available:%zd tcp-read:%zd serial-write:%zd\n", maxToSerial, tcp_got, serial_sent);
      }
    }
#endif

  // determine maximum output size "fair TCP use"
  // client.availableForWrite() returns 0 when !client.connected()
  size_t maxToTcp = 0;
  for (int i = 0; i < MAX_SRV_CLIENTS; i++)
    if (serverClients[i]) {
      size_t afw = serverClients[i].availableForWrite();
      if (afw) {
        if (!maxToTcp) {
          maxToTcp = afw;
        } else {
          maxToTcp = std::min(maxToTcp, afw);
        }
      } else {
        // warn but ignore congested clients
        Serial.println("one client is congested");
      }
    }

  //check UART for data
  size_t len = std::min((size_t)slen, maxToTcp);
  len = std::min(len, (size_t)STACK_PROTECTOR);
  if (len) {
    size_t serial_got = slen;
    slen=0;
    // push UART data to all connected telnet clients
    for (int i = 0; i < MAX_SRV_CLIENTS; i++)
      // if client.availableForWrite() was 0 (congested)
      // and increased since then,
      // ensure write space is sufficient:
      if (serverClients[i].availableForWrite() >= serial_got) {
        size_t tcp_sent = serverClients[i].write(sdata, serial_got);
        if (tcp_sent != len) {
          Serial.printf("len mismatch: available:%zd serial-read:%zd tcp-write:%zd\n", len, serial_got, tcp_sent);
        }
        //Clearing the data
      memset(sdata,0,serial_got);
      }
  len = 0;
  }
}
