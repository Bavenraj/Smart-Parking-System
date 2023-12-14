// SMART PARKING SYSTEM MASTERPIECE
// CHIN FU VIN, BAVENRAJ A/L PARTIPAN, CHEW KAI LIANG

#define BLYNK_TEMPLATE_ID "TMPL6TF2lVCVN"
#define BLYNK_TEMPLATE_NAME "Parking"
#define BLYNK_AUTH_TOKEN "3K3CHUc45zXwsFVWYnX_cNEmiME3rk5c"

#include "DurianBlynkESP8266.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Servo.h> 
#include <SoftwareSerial.h>

SoftwareSerial EspSerial(2, 3); // RX, TX
LiquidCrystal_I2C lcd(0x27,16,2);

#define ESP8266_BAUD 9600
char auth[] = BLYNK_AUTH_TOKEN;
#ifdef BLYNK_TEMPLATE_ID && BLYNK_TEMPLATE_NAME
char tempID[] = BLYNK_TEMPLATE_ID; 
char deviceName[] = BLYNK_TEMPLATE_NAME; 
#else
char tempID[5]; 
char deviceName[5]; 
#endif

#define BLYNK_GREEN     "#23C48E"
#define BLYNK_RED       "#D3435C"

char ssid[] = "G678";
char pass[] = "abc123adc";

DBlynk DBlynk(&EspSerial);
Servo servo;

int irEntrance = 10;  // Pin connected to the entrance IR sensor
int irExit = 12;      // Pin connected to the exit IR sensor

int greenLED = 9;     // Pin connected to the green LED
int redLED = 8;       // Pin connected to the red LED

int buzzer = 4;       // Pin connected to the buzzer

int parkingLotCount = 4;    // Total number of parking lots
int availableParkingLots = parkingLotCount;  // Initially, all parking lots are available

int gateOpen = 50;   // Gate barrier angle when open
int gateClose = 155;   // Gate barrier angle when closed
int totalCars = 0;

void setup() 
{
  Serial.begin(9600);
  Serial.println("Remaining Parking Lots: " + String(availableParkingLots));

  // Initialize the LCD
  lcd.init();
  lcd.clear();
  lcd.backlight();
  
  //waiting wifi and display on LCD
  lcd.print("Wifi Connecting");
  lcd.setCursor(0,1);
  lcd.print("...");

  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  delay(10);
  
  //Connect to Blynk
  DBlynk.begin(BLYNK_TEMPLATE_ID, BLYNK_TEMPLATE_NAME, auth, ssid, pass); 

  //Wifi connected and display on LCD
  lcd.print("Done!");

  //delay 1 second for LCD display
  delay(1000);

  servo.attach(7);
  servo.write(gateClose);
  delay(500);

  pinMode(irEntrance, INPUT);
  pinMode(irExit, INPUT);

  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
  
  pinMode(buzzer, OUTPUT);

  lcd.setCursor(0, 0);
  lcd.print("Welcome to Smart ");
  lcd.setCursor(0, 1);
  lcd.print("Parking System"); 
  delay(2500);
  lcd.clear();

  updateLCD();
  //DBlynk.virtualWrite(V3, HIGH);
  //DBlynk.virtualWrite(V4, HIGH);
  DBlynk.setProperty(V3, "color", BLYNK_RED); //set Blynk widget LED
  DBlynk.setProperty(V4, "color", BLYNK_RED); //set Blynk widget LED
}

void barControl(int angle)
{
  servo.write(angle);
  //delay(500);
  //servo.detach();
}

void loop() 
{
  DBlynk.run(); // Run Blynk
  DBlynk.virtualWrite(V0, "Available Slots");
  DBlynk.virtualWrite(V1, availableParkingLots);
  int entranceState = digitalRead(irEntrance);
  int exitState = digitalRead(irExit);

  // Close the gate if nothing is detected
  barControl(gateClose);

// Light up the green LED if there are available parking lots, otherwise light up the red LED
      if (availableParkingLots > 0) 
      {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
      } else {
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
      }

  // Check if a car is detected at the entrance or exit
  if (entranceState == LOW || exitState == LOW) 
  {
    // Entrance Part
    if (entranceState == LOW) 
    {
      // Open the gate
      barControl(gateOpen);
      tone(buzzer, 1000);
      totalCars ++;
      DBlynk.virtualWrite(V2, totalCars);

      // Wait until the car has fully passed through
      while (entranceState == LOW || exitState == LOW) 
      {
        entranceState = digitalRead(irEntrance);
        exitState = digitalRead(irExit);
      }

      // Silence the buzzer
      noTone(buzzer);

      // Control the barrier
      delay(2000);
      barControl(gateClose);
      delay(1000);

      // Update the available parking lots count
      if (availableParkingLots > 0) 
      {
        availableParkingLots--;
      }

      // Update the sensor states
      entranceState = digitalRead(irEntrance);
      exitState = digitalRead(irExit);

      // Update the LCD display
      updateLCD();
      Serial.println("Car Entered");
      Serial.println("Remaining Parking Lots: " + String(availableParkingLots));
      DBlynk.setProperty(V3, "color", BLYNK_GREEN);
      delay(500);
      DBlynk.setProperty(V3, "color", BLYNK_RED);
      //DBlynk.virtualWrite(V3, HIGH);
      DBlynk.virtualWrite(V0, "Available Slots");
      DBlynk.virtualWrite(V1, availableParkingLots);
    }

    // Exit Part
    if (exitState == LOW) 
    {
      // Open the gate
      barControl(gateOpen);
      tone(buzzer, 2000); 

      // Wait until the car has fully exited
      while (entranceState == LOW || exitState == LOW) 
      {
        entranceState = digitalRead(irEntrance);
        exitState = digitalRead(irExit);
      }

      // Silence the buzzer
      noTone(buzzer);

      // Control the barrier
      delay(2000);
      barControl(gateClose);
      delay(1000);

      // Update the available parking lots count
      if (availableParkingLots < parkingLotCount) 
      {
        availableParkingLots++;
      }

      // Update the sensor states
      entranceState = digitalRead(irEntrance);
      exitState = digitalRead(irExit);


      // Update the LCD display
      updateLCD();
      Serial.println("Car Exited");
      Serial.println("Remaining Parking Lots: " + String(availableParkingLots));
      DBlynk.setProperty(V4, "color", BLYNK_GREEN);
      delay(500);
      DBlynk.setProperty(V4, "color", BLYNK_RED);
      //DBlynk.virtualWrite(V4, HIGH);
       //set Blynk widget LED to Red colour
      DBlynk.virtualWrite(V0, "Available Slots");
      DBlynk.virtualWrite(V1, availableParkingLots);
    }
  }
  delay(500);
}

void updateLCD() 
{
  // Print the remaining slots
  lcd.setCursor(0, 0);
  lcd.print("Welcome to Smart ");
  lcd.setCursor(0, 1);
  lcd.print("Parking System!"); 
  delay(2500);
  lcd.clear();


  lcd.setCursor(0,0);
  lcd.print("Available Slots:");
  lcd.setCursor(0,1);
  lcd.print(availableParkingLots);
}
