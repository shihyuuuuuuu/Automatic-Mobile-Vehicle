#include <BRCClient.h>
#include <SPI.h>
#include <RFID.h>
#define nowVolt 8.05
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCLK 13
#define SPI_SS 10
#define MFRC522_RSTPD 9
//#define wifi
int mode = 0;

RFID rfid(SPI_SS, MFRC522_RSTPD);

static uint8_t status;
static uint8_t tagSN[4];

int x, y;
char type;
CommMsg msg;
MapMsg map_d;
bool RFIDGET = false;

float pwm(float needVolt);
bool readTag();
//馬達PWM腳位
const byte ENL = 6;   //左輪馬達的 PWM 轉速控制
const byte ENR = 5;   //右輪馬達的 PWM 轉速控制

//超音波腳位
const byte lEcho = 3;
const byte fEcho = 14;
const byte rEcho = 16;
const byte lTrig = 4;
const byte fTrig = 15;
const byte rTrig = 17;

//74HC595腳位
const byte dataPin = 7;
const byte latchPin = 8;
const byte clockPin = 9;
 
byte speed = pwm(2.5);//9.8*(110/255) = 4.22V

//安全距離
#define front 13.0
#define left 7.0
#define right 7.0
#define UNO

#ifdef UNO
  #define UART_RX 2
  #define UART_TX 18
#else
  #define UART_RX 10
  #define UART_TX 2
#endif

#if !defined(UNO) && defined(USE_HARDWARE_SERIAL)
 BRCClient brcClient(&HW_SERIAL);
#else
 BRCClient brcClient(UART_RX, UART_TX);
#endif

#define AP_SSID    "AndroidAPP"
#define AP_PASSWD  "19961214"
#define TCP_IP     "192.168.43.1"
/*#define AP_SSID    "programtheworld"
#define AP_PASSWD  "screamlab"
#define TCP_IP     "192.168.150.11"*/    
#define TCP_PORT   5000
#define MY_COMM_ID 0x87

//存三邊危險與否的陣列
int LFR[3] = {0};
int count;
bool registerOK = false;
bool recieveGo = false;
bool start = false;
const byte IC[9] = {
    B01010000,//forward
    B00101000,//backward
    B01001000,//left
    B00110000,//right
    B11010000,//miniLeft
    B01010100,//miniRight
    B00000000,//stop
    B00100000,//right back
    B00001000,//left back
  };

//與障礙物距離
float lDistance;//左
float fDistance;//前
float rDistance;//右

void forward()//前進
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, IC[0]);
  digitalWrite(latchPin, HIGH);
  digitalWrite(latchPin, LOW);
}

void backward()//後退
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, IC[1]);
  digitalWrite(latchPin, HIGH);
  digitalWrite(latchPin, LOW); 
}

void Left()//左轉
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, IC[2]);
  digitalWrite(latchPin, HIGH);
  digitalWrite(latchPin, LOW);
}
 
void Right()//右轉
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, IC[3]);
  digitalWrite(latchPin, HIGH);
  digitalWrite(latchPin, LOW);
}

void stop()//停止
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, IC[6]);
  digitalWrite(latchPin, HIGH);
  digitalWrite(latchPin, LOW);
}

void miniLeft()
{
  Left();
  delay(27);
  forward();
  delay(30);
  Right();
  delay(15);
  forward();
}

void miniRight()
{
  Right();
  delay(27);
  forward();
  delay(30);
  Left();
  delay(15);
  forward();
}

void backAndLeft()
{
  stop();
  delay(100);
  backward();
  delay(275);
  stop();
  delay(100);
  Left();
  delay(55);
  forward();
  delay(100);
}

void backAndRight()
{
  stop();
  delay(100);
  backward();
  delay(275);
  stop();
  delay(100);
  Right();
  delay(55);
  forward();
  delay(100);
}

float fDis()//前方障礙距離
{
  float duration, distance;
  digitalWrite(fTrig, HIGH);
  delayMicroseconds(5);
  digitalWrite(fTrig, LOW);
  
  duration = pulseIn(fEcho, HIGH,7000);
  distance = (duration == 0) ? 3000 : duration/58.0;
  Serial.print("  3 = ");
  Serial.print(distance);
  Serial.print(" cm");
  return distance;
}

float lDis()//左方障礙距離
{
  float duration, distance;
  digitalWrite(lTrig, HIGH);
  delayMicroseconds(5);
  digitalWrite(lTrig, LOW);
  
  duration = pulseIn(lEcho, HIGH, 7000);
  distance = (duration == 0) ? 3000 : duration/58.0;
  Serial.print("1 = ");
  Serial.print(distance);
  Serial.print(" cm");
  return distance;
}

float rDis()//右方障礙距離
{
  float duration, distance;
  digitalWrite(rTrig, HIGH);
  delayMicroseconds(5);
  digitalWrite(rTrig, LOW);
  
  duration = pulseIn(rEcho, HIGH, 7000);
  distance = (duration == 0) ? 3000 : duration/58.0;
  Serial.print("  5 = ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

float pwm(float needVolt){
  return (needVolt/nowVolt)*255;
}

bool readTag()
{
  uint8_t status, snBytes, sn[MAXRLEN];
  uint16_t card_type; 
  if ((status = rfid.findTag(&card_type)) == STATUS_OK && card_type == 1024){
    if ((status = rfid.readTagSN(sn, &snBytes)) == STATUS_OK) 
    {
      tagSN[0] = sn[0];
      tagSN[1] = sn[1];
      tagSN[2] = sn[2];
      tagSN[3] = sn[3];
      rfid.piccHalt();
      return true;
    }
  }
  return false;
}

void getRFID(){
  CommMsg msg;
  if(readTag()){
    RFIDGET = true;
    digitalWrite(19, HIGH);
    delay(50);
    digitalWrite(19, LOW);
    brcClient.requestMapData(tagSN, "I am 87");
    brcClient.receiveMessage(&msg);
    map_d = rawDataToMapMsg(msg.buffer);
    x = map_d.x;
    y = map_d.y;
    type = map_d.type;

    if(type == (char)MAP_PARK_2){
      brcClient.sendToClient(0xA2,"9487");
      stop();
      msg = {.type = MSG_ROUND_COMPLETE, .ID = 0x01};
      brcClient.sendMessage(&msg);
      brcClient.endBRCClient();
      while(1){
        stop();
        delay(1);
      }
    }

  }
  return;
}

void disUpdate()
{
  float l[2], f[2], r[2];
  int i;
  for(i = 0; i < 2; ++i){
    l[i] = lDis();
    f[i] = fDis();
    r[i] = rDis();
    delay(5);
  }

  lDistance = fabs(l[0]-l[1])<2?l[0]:lDis();
  fDistance = fabs(f[0]-f[1])<2?f[0]:fDis();
  rDistance = fabs(r[0]-r[1])<2?r[0]:rDis();
}

bool crossRoad(int mode){
  if(mode == 1){
    if(rDistance > 30 && fDistance > 30 && lDistance < 15){//右方前方岔路
          forward();
          delay(225);
          stop();
          delay(100);
          Right();
          delay(330);
          stop();
          delay(100);
          forward();
          delay(225);
          return true;
    }else if(rDistance > 30 && lDistance > 30 && fDistance < 30){//右方左方岔路
          forward();
          delay(225);
          stop();
          delay(100);
          Right();
          delay(330);
          stop();
          delay(100);
          forward();
          delay(225);
          return true;
    }else if(lDistance > 30 && fDistance > 30 && rDistance < 15){//前方左方岔路
          while(rDistance > 15 || lDistance > 15){
            disUpdate();
            if(rDistance > 10)
              if(rDistance > 15)
                backAndRight();
              else
                miniRight();
            else if(rDistance < 7){
              if(rDistance < 4)
                backAndLeft();
              else
                miniLeft();
            }else{
              forward();
              delay(50);
            }
          }
      return true;
    }
    return false;
  }else if(mode == 0){
    if(rDistance > 30 && fDistance > 30 && lDistance < 15){//右方前方岔路
          while(rDistance > 15 || lDistance > 15){
            disUpdate();
            if(lDistance > 10)
              if(lDistance > 15)
                backAndLeft();
              else
                miniLeft();
            else if(lDistance < 7){
              if(lDistance < 4)
                backAndRight();
              else
                miniRight();
            }else{
              forward();
              delay(50);
            }
          }
      return true;
    }else if(rDistance > 30 && lDistance > 30 && fDistance < 30){//右方左方岔路
          forward();
          delay(225);
          stop();
          delay(100);
          Left();
          delay(330);
          stop();
          delay(100);
          forward();
          delay(225);
          return true;
    }else if(lDistance > 30 && fDistance > 30 && rDistance < 15){//前方左方岔路
          forward();
          delay(225);
          stop();
          delay(100);
          Left();
          delay(330);
          stop();
          delay(100);
          forward();
          delay(225);
          return true;
    }
    return false;
  }
}

void swing(){
  RFIDGET = false;
  stop();
  getRFID();
  stop();
  getRFID();
  stop();
  getRFID();
  backward();
  delay(160);
  stop();
  getRFID();
  stop();
  getRFID();
  stop();
  getRFID();
  while(!RFIDGET){
    forward();
    delay(130);
    stop();
    getRFID();
    stop();
    getRFID();
    stop();
    getRFID();
    backward();
    delay(165);
    stop();
    getRFID();
    stop();
    getRFID();
    stop();
    getRFID();
    bool mode;
    disUpdate();
    if(lDistance > rDistance)
      mode = true;
    else
      mode = false;
    if(mode){
      Left();
      delay(140);
      stop();
      getRFID();
      stop();
      getRFID();
      stop();
      getRFID();
    }else{
      Right();
      delay(140);
      stop();
      getRFID();
      stop();
      getRFID();
      stop();
      getRFID();
    }
  }
}

void setup() {
  Serial.begin(9600);
  
  pinMode(ENL, OUTPUT);
  pinMode(ENR, OUTPUT);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  pinMode (lEcho, INPUT);
  pinMode (fEcho, INPUT);
  pinMode (rEcho, INPUT);
  pinMode (lTrig, OUTPUT);
  pinMode (fTrig, OUTPUT);
  pinMode (rTrig, OUTPUT);
  pinMode(19, OUTPUT);

  #ifdef wifi
  brcClient.begin(9600);
  brcClient.beginBRCClient(AP_SSID, AP_PASSWD, TCP_IP, TCP_PORT);
  delay(2000);
  while(!brcClient.registerID(MY_COMM_ID))
    delay(500);
  registerOK = true;
  Serial.println("ID register OK");
  #endif

  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000L, MSBFIRST, SPI_MODE3));

  rfid.begin();
}


void loop() {
    CommMsg msg;
    char buf[40];
    #ifdef wifi
    recieveGo = brcClient.receiveMessage(&msg);
   
    if(registerOK && recieveGo && msg.type == MSG_ROUND_START || start){
    #endif
      
    //if(registerOK && recieveGo && msg.ID == (char)0xA1 || start){
      /*if(msg.type == (char)MSG_ROUND_END){
        brcClient.endBRCClient();
        while(1){
          stop();
          delay(1);
        }
      }*/
      /*if(fDistance < 15 && rDistance < 20 && lDistance < 20){
        CommMsg msg;
        msg.type = MSG_ROUND_COMPLETE;
        brcClient.sendMessage(&msg); 
        brcClient.endBRCClient();
        while(1){
          stop();
          delay(100);
        }
      }*/
    start = true;
    disUpdate();

    int i;
    for(i = 0; i < 3; ++i)
      LFR[i] = 0;

   if(lDistance < left)   LFR[0] = 1;
   if(fDistance < front)  LFR[1] = 1;
   if(rDistance < right)  LFR[2] = 1;

   count = 0;//紀錄有幾邊危險
   for(i = 0; i < 3; ++i){
     if(LFR[i] == 1)  count++;
    }

    /*每次loop初始化速度*/
    analogWrite(ENL, pwm(4.4));//4.458
    analogWrite(ENR, pwm(4.0));

    switch(count){
    case 0://3邊安全，直走
      if(!crossRoad(mode)){
        forward();
        delay(50);
      }
      break;
    case 1://1邊危險
      if(LFR[0] == 1){
        if(!crossRoad(mode)){
          if(lDistance > 5)
            miniRight();
          else
            do{
              backAndRight();
              disUpdate();
            }while(lDistance < 5);
        }
      }else if(LFR[1] == 1){
        stop();
        delay(100);
        disUpdate();
        if(!crossRoad(mode)){
          if(rDistance < 20 && lDistance < 20){
            if(rDistance > lDistance){
              Right();
              delay(570);
            }else{
              Left();
              delay(570);
            }
            stop();
            delay(100);
            disUpdate();
            if(abs(lDistance - rDistance) > 8){
              if(lDistance > rDistance) Left();
              else Right();
              delay(80);
            }
            swing();
            forward();
            delay(100);
          }else{
            if(rDistance > lDistance){
              Right();
              delay(350);
            }else{
              Left();
              delay(320);
            }
            stop();
            delay(100);
            forward();
            delay(225);
          }
        }
      }else if(LFR[2] == 1){
        if(!crossRoad(mode)){
          if(rDistance > 5)
            miniLeft();
          else
            do{
              backAndLeft();
              disUpdate();
            }while(rDistance < 5);
        }
      }
      break;
    case 2://2邊危險
      stop();
      delay(100);
      if((LFR[0] == 1 && LFR[1] == 1)) {
          if(rDistance < 20){
              Right();
              delay(570);
              stop();
              delay(100);
              disUpdate();
              if(abs(lDistance - rDistance) > 8){
                if(lDistance > rDistance) Left();
                else  Right();
                delay(80);
              }
              swing();
              forward();
              delay(100);
          }else{
            Right();
            delay(350);
            stop();
            delay(100);
            forward();
            delay(225);
         }
      }else if(LFR[1] == 1 && LFR[2] == 1){
          if(lDistance < 20){
              Left();
              delay(570);
              stop();
              delay(100);
              disUpdate();
              if(abs(lDistance - rDistance) > 8){
                if(lDistance > rDistance) Left();
                else  Right();
                delay(80);
              }
              swing();
              forward();
              delay(100);
          }else{
            Left();
            delay(320);
            stop();
            delay(100);
            forward();
            delay(225);
         }
      }
      break;
    case 3://3邊都危險
      forward();
      delay(90);
      break;
    default:
      forward();
      delay(90);
      break;
  }
  delay(2);
  #ifdef wifi
  }
  #endif
}

