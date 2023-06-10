#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)



#include <HardwareSerial.h>
#include <TinyGPS++.h>
#define RXD2 16
#define TXD2 17
TinyGPSPlus gps;
double bike_speed;


#define input1 26
#define input2 27



#define red 19
#define yellow 18
#define green 5
#define buzzer 4
#define switchRelay 0  



#define LED_BUILTIN 2

int helmet = 0;
int stand = 0;
int wheel = 0;
bool handleflag = 0;
bool motion = 0;
// Timers for Helmet and wheel
unsigned long helmetPreviousTime = 0;

unsigned long wheelPreviousTime = 0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  int IR;
}struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Create an array with all the structures
struct_message boardsStruct[3]= {board1, board2, board3};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  digitalWrite(LED_BUILTIN,HIGH);
  
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  

  boardsStruct[myData.id-1].IR = myData.IR;
  //helmet
  if(myData.id == 1){
    helmetPreviousTime = millis()/1000; 
  }
  if(myData.id == 3){
    wheelPreviousTime = millis()/1000; 
  }

  

  Serial.printf("IR value: %d \n", boardsStruct[myData.id-1].IR);
  Serial.println();
  digitalWrite(LED_BUILTIN,LOW);
}


void O_Print(String p , int row , int column){
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(row, column);
  // Display static text
  display.println(p);
  display.display();
}


 
void setup() {
  //Initialize Serial Monitor

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2); //gps baud



  Serial.begin(115200);
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(switchRelay, OUTPUT);

  digitalWrite(red, HIGH);
  digitalWrite(yellow, LOW);
  digitalWrite(green, LOW);
  digitalWrite(buzzer, LOW);
  digitalWrite(switchRelay, HIGH);

  pinMode(input1,INPUT);
  pinMode(input2,INPUT);

  pinMode(LED_BUILTIN,OUTPUT);
  
  boardsStruct[0].id = 1;
  boardsStruct[1].id = 2;
  boardsStruct[2].id = 3;
  boardsStruct[2].IR = 1;
  boardsStruct[0].IR = 0;
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packe r info
  esp_now_register_recv_cb(OnDataRecv);
}
 
 
unsigned long previoustime = 0;
unsigned long currenttime = 0;
unsigned long currentTime = 0;

unsigned long oledprevioustime = 0;
unsigned long oledcurrenttime = 0;
int timerarray[4] = {1,1,1,1};
String printstring[4] = {"Helmet","Stand","Wheel","Handle"};
int timerindex = 0;
bool isCaution = 0;

void loop() {
  //Code for checking if signal was received inside allowed time window

  
  currentTime = millis()/1000;
  if(currentTime-helmetPreviousTime > 6){
    boardsStruct[0].IR = 0;
  }
  
  
  //Remanining Code below
  helmet = boardsStruct[0].IR;
  stand = digitalRead(input1);
  //stand = boardsStruct[1].flag;
  wheel = 0;
  //wheel = 0;

  handleflag = digitalRead(input2); 

 
  bool recebido = false;
  while (Serial1.available()) {
     char cIn = Serial1.read();
     recebido = gps.encode(cIn);
  }
   if (gps.location.isUpdated() && gps.altitude.isUpdated())
  {
    Serial.print("SPEED= ");  Serial.println(gps.speed.kmph(), 6);
    }
  bike_speed = gps.speed.kmph();
  Serial.print("SPEED= ");  Serial.println(gps.speed.kmph(), 6);
  Serial.print(gps.location.lat(), 6);
  motion = bike_speed>20;

  //Logic for red light and switch relay 
  
  if(helmet == 0 || stand == 0 || wheel == 1){ 
    
      //IR = LOW = ok , High = ERROR ,stand =0 = error 
    digitalWrite(red,HIGH);
    digitalWrite(switchRelay,LOW);
  }else{
    digitalWrite(red,LOW);
    digitalWrite(switchRelay,HIGH);
  }


  //logic for green light

  if(helmet == 1 && stand == 1 && wheel==0 && handleflag == 1){             // if all crucial things are present  All IR = 0  // and handle is also ok = 1
    digitalWrite(green,HIGH);
  }else{
    digitalWrite(green,LOW);
  }
  




  //logic for yellow light and buzzer
  if(helmet == 1 && stand == 1 && wheel==0 && handleflag == 0){
    digitalWrite(yellow,HIGH);
      if(motion){
         currenttime = millis()/1000;
         digitalWrite(buzzer,HIGH);
         previoustime = currenttime;
      }else{
        currenttime = millis()/1000;

        if(currenttime-previoustime > 3){
          digitalWrite(buzzer,LOW);
        }
      }
    
  }else{
    digitalWrite(yellow,LOW);
    currenttime = millis()/1000;
        if(currenttime-previoustime > 3){
          digitalWrite(buzzer,LOW);
        }
  }

// setting flaGS
 if(helmet == 0){
     timerarray[0] = 1;   
  }else{
    timerarray[0] = 0;
  }

  if(stand == 0){
     timerarray[1] = 1;   
  }else{
    timerarray[1] = 0;
  }


  if(wheel == 1){
     timerarray[2] = 1;    
  }else{
    timerarray[2] = 0;
  }


  if(handleflag == 0){
     timerarray[3] = 1;
  }else{
    timerarray[3] = 0;
  }

  if(helmet == 0 || stand == 0 || wheel == 1 || handleflag ==0){ 
    isCaution = 1;
  }else{
    isCaution = 0;
  }



  // Logic for Printing Strings on the Oled Display @Author : Muhammad Abdullah

oledcurrenttime = millis()/1000;


if(isCaution && oledcurrenttime-oledprevioustime >1){
    Serial.println("In Condition");
    Serial.print("Timer Index is: ");
    Serial.println(timerindex);
  for(int i=0;i<4;i++){
    Serial.println("i is:");
    Serial.println(timerarray[i]);
    Serial.println(timerarray[timerindex]);
     if( timerarray[timerindex] == 1){
      
       oledcurrenttime = millis()/1000;
       Serial.println(printstring[timerindex]);
       O_Print(printstring[timerindex],1,1);
       oledprevioustime = currenttime;
       if(timerindex == 3){
         timerindex = 0;
         Serial.println("Setting");
       }else{
           Serial.println("Incrementing");
           timerindex++;
        }
        break;
      }
      if(timerarray[timerindex] == 0){
        if(timerindex == 3){
         timerindex = 0;
         Serial.println("Setting");
       }else{
           Serial.println("Incrementing");
           timerindex++;
        }
      }
    
  } 
}


// if all Ok

if(isCaution == 0) {
  O_Print("All Ok",1,1);
 }

}
