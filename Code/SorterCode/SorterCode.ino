/*
   Tijmen Van Der Beek
   5/25/2021
   SorterCode
   This program is going to run the candy sorting and dispensing machine.
   The goal is to sort candy by it's color into seperate cases and dispense it with a nfc reader.
   
   Peripherals:
   Small OLed Display
   Stepper Motor
   RGB Color Sensor
   RGB Led
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Adafruit_TCS34725.h>

#define CaliOffset 0
#define CaliClear 1
#define CaliRGB 2

void padding(char* str, char* format, int padding);

//--------------------------------PINS---------------------------------//
//LED
#define LEDPinR 9
#define LEDPinG 10
#define LEDPinB 11

//Button
#define ButtonPin 2

//Display
#define displaySCL A5 //can't Change
#define displaySDA A4 //can't Change
#define OLED_RESET -1 //reset pin //can't Change

#define SCREEN_WIDTH 128 //display width
#define SCREEN_HEIGHT 32 //display height
#define SCREEN_ADDRESS 0x3C

//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Stepper Motor

//Color Sensor
#define sensorSCL A5 //can't Change
#define sensorSDA A4 //can't Change

Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

//StartUp Bitmap
static const unsigned char PROGMEM startup_bmp[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x0f, 0xc0, 0x06, 0xc1, 0x83, 0x33, 0xfc, 0x7f, 0x88, 0x04, 0x80, 0x4c, 0xff, 0x00, 
    0x00, 0x80, 0x2f, 0x80, 0x1e, 0xf1, 0x83, 0x33, 0xfe, 0x7f, 0xdc, 0x0e, 0xc0, 0xcc, 0xff, 0x80, 
    0x02, 0xc0, 0x6f, 0x00, 0x3e, 0xf9, 0x83, 0x33, 0xfe, 0x7f, 0xce, 0x1c, 0xe1, 0xcc, 0xff, 0x80, 
    0x06, 0xe0, 0xee, 0x00, 0x38, 0x39, 0x83, 0x30, 0x06, 0x00, 0xc7, 0x38, 0x73, 0xcc, 0x01, 0x80, 
    0x00, 0xf1, 0xec, 0x00, 0x70, 0x18, 0x03, 0x00, 0x06, 0x00, 0xc2, 0x70, 0x3f, 0xc0, 0x01, 0x80, 
    0x00, 0xf3, 0xe8, 0x00, 0x70, 0x01, 0xff, 0x30, 0x06, 0x00, 0xc0, 0xe0, 0x1e, 0xcc, 0x01, 0x80, 
    0x00, 0xe7, 0xe0, 0x00, 0x70, 0x01, 0xff, 0x33, 0xfe, 0x7f, 0xc0, 0xc0, 0x0c, 0xcc, 0xff, 0x80, 
    0x00, 0x00, 0x00, 0x00, 0x70, 0x19, 0x83, 0x33, 0xfc, 0x7f, 0x80, 0xc0, 0x00, 0xcc, 0xff, 0x00, 
    0x00, 0x7f, 0xc0, 0x00, 0x38, 0x39, 0x83, 0x33, 0x00, 0x60, 0x00, 0xc0, 0xc0, 0xcc, 0xc0, 0x00, 
    0x00, 0x3f, 0xc0, 0x00, 0x3e, 0xf9, 0x83, 0x33, 0x00, 0x60, 0x00, 0xc0, 0xc0, 0xcc, 0xc0, 0x00, 
    0x00, 0x1f, 0xc0, 0x00, 0x1e, 0xf1, 0x83, 0x33, 0x00, 0x60, 0x00, 0xc0, 0xc0, 0xcc, 0xc0, 0x00, 
    0x00, 0x0f, 0x80, 0x00, 0x06, 0xc1, 0x83, 0x33, 0x00, 0x60, 0x00, 0xc0, 0xc0, 0xcc, 0xc0, 0x00, 
    0x00, 0x07, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x03, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x60, 0x00, 0x36, 0x72, 0x75, 0x13, 0x0e, 0xaa, 0x26, 0x30, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x60, 0x00, 0x45, 0x45, 0x25, 0x94, 0x04, 0xab, 0x28, 0x40, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x60, 0x00, 0x46, 0x67, 0x25, 0x55, 0x84, 0xea, 0xab, 0x30, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0xe0, 0x00, 0x45, 0x45, 0x25, 0x34, 0x84, 0xaa, 0x69, 0x08, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x60, 0x00, 0x35, 0x75, 0x25, 0x13, 0x04, 0xaa, 0x26, 0x30, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

//Variables
bool buttonPressed = false;
int debounceTime = 0;
int32_t offset[] = {0, 0, 0, 0};
float multiplier[] = {1, 1, 1, -.757};
float ratio[] = {0, 0, 0, 0};
uint16_t caliNum = 0; //number of times rgb has been calibrated

void setup() {
  //LED setup
  pinMode(LEDPinR, OUTPUT);
  pinMode(LEDPinG, OUTPUT);
  pinMode(LEDPinB, OUTPUT);

  //Button Setup
  pinMode(ButtonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ButtonPin), buttonISR, FALLING);
  
  //Display Setup
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  /*
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.drawBitmap(0, 0, startup_bmp, 128, 32, 1);
  display.display();
  delay(2000);

  display.clearDisplay();
  display.drawPixel(1, 1, SSD1306_WHITE);
  display.setFont(&FreeMono9pt7b);
  display.drawChar(30, 30, 'H', 1, 0, 1);
  display.display();
  */

  if (colorSensor.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  interrupts();
  CalibrateColorSensor();
}

void loop() {
  uint16_t t[11][4];
  setLed(0, 100, 0);
  waitForUser();
  runSensor(t, false);
  printTable("Color Table", t);
  //float r, g, b, c;//, colorTemp, lux;
  
  //colorSensor.getRawData(&r, &g, &b, &c);
  //colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  //lux = tcs.calculateLux(r, g, b);

  //Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  //Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  //Serial.print("Red:   "); Serial.print(r); Serial.print(" ");
  //Serial.print("Green: "); Serial.print(g); Serial.print(" ");
  //Serial.print("Blue:  "); Serial.print(b); Serial.print(" ");
  //Serial.print("Clear: "); Serial.print(c); Serial.print(" ");
  //Serial.println(" ");
  
}

//---------------------------------Functions--------------------------------//
/*
   setLed
   sets the rgb led color
   Input:
   int red value (0-255)
   int green value (0-255)
   int blue value (0-255)
*/
void setLed(int rVal, int gVal, int bVal){
  analogWrite (LEDPinR, rVal);
  analogWrite (LEDPinG, gVal);
  analogWrite (LEDPinB, bVal);
}

void waitForUser(){
  Serial.println("Waiting For button!");
  while(digitalRead(ButtonPin) == 1){}
  Serial.println("Pressed!");
  buttonPressed == false;
}

//ISRs
/*
   buttonISR
   sets clear_tags to true and sets led to pink
*/
void buttonISR() {
  setLed(200, 100, 0);
  if(millis() - debounceTime > 100){
    delay(5);
    if(digitalRead(ButtonPin) == 0){
      buttonPressed = true;
    }
  }
  debounceTime = millis();
}

//Test Functions
void CalibrateColorSensor(){
  uint16_t t[11][4];
  //Black (49, 48, 49)
  setLed(0, 0, 0);
  waitForUser();
  calibrateColorSensor(49, 48, 49, t, CaliOffset);
  printTable("Black <--------", t);

  //White (229, 226, 218)
  setLed(255, 100, 240);
  waitForUser();
  calibrateColorSensor(229, 226, 218, t, CaliClear);
  printTable("White <--------", t);
  /*
  //Red (164, 46, 55)
  setLed(255, 0, 0);
  waitForUser();
  calibrateColorSensor(164, 46, 55, t, CaliRGB);
  printTable("Red <--------", t);
  
  //Green (127, 172, 110)
  setLed(0, 100, 0);
  waitForUser();
  calibrateColorSensor(127, 172, 110, t, CaliRGB);
  printTable("Green <--------", t);
  
  //Blue (1, 93, 135)
  setLed(0, 0, 240);
  waitForUser();
  calibrateColorSensor(1, 93, 135, t, CaliRGB);
  printTable("Blue <--------", t);*/
}

/*
  order matter offset -> clear_multiplier -> rgb_multiplier
  offset: 0
  clear_multiplyer: 1
  rgb_multiplyer: 2
*/
void calibrateColorSensor(int trueR, int trueG, int trueB, uint16_t table[11][4], uint8_t type){
  runSensor(table, true);
  
  float rVal = table[10][0];
  float gVal = table[10][1];
  float bVal = table[10][2];
  float cVal = table[10][3];
  float avgSensorVal;
  float avgActualVal;
  
  
  if(type == CaliOffset){
    offset[0] = rVal - trueR;
    offset[1] = gVal - trueG;
    offset[2] = bVal - trueB;

    multiplier[0] = ((float) trueR) / rVal;
    multiplier[1] = ((float) trueG) / gVal;
    multiplier[2] = ((float) trueB) / bVal;
    
    offset[3] = cVal;
  }
  else if(type == CaliClear){
    calcOffset(&rVal, &gVal, &bVal, &cVal);
    
    avgSensorVal = (float (rVal + gVal + bVal)) / 3;
    avgActualVal = (float (trueR + trueG + trueB)) / 3;
    
    multiplier[3] = log(avgActualVal / (avgSensorVal * cVal)) / log(cVal);
  }
  else if(type == CaliRGB){
    calcOffset(&rVal, &gVal, &bVal, &cVal);
    calcSaturation(&rVal, &gVal, &bVal, &cVal);
    
    ratio[0] *= caliNum;
    ratio[1] *= caliNum;
    ratio[2] *= caliNum++;
    
    ratio[0] += trueR / ((rVal * rVal) - (trueR * rVal) + trueR);
    ratio[1] += trueG / ((gVal * gVal) - (trueG * gVal) + trueG);
    ratio[2] += trueB / ((bVal * bVal) - (trueB * bVal) + trueB);
    
    ratio[0] /= caliNum;
    ratio[1] /= caliNum;
    ratio[2] /= caliNum;
  }

  char s[100] = "";
  Serial.println();
  Serial.print("Offsets: ");
  Serial.print(offset[0]); Serial.print(", ");
  Serial.print(offset[1]); Serial.print(", ");
  Serial.print(offset[2]); Serial.print(", ");
  Serial.println(offset[3]);
  Serial.print("Multipliers: ");
  Serial.print(multiplier[0], 6); Serial.print(", ");
  Serial.print(multiplier[1], 6); Serial.print(", ");
  Serial.print(multiplier[2], 6); Serial.print(", ");
  Serial.println(multiplier[3], 6);
  Serial.print("Ratios: ");
  Serial.print(ratio[0], 6); Serial.print(", ");
  Serial.print(ratio[1], 6); Serial.print(", ");
  Serial.print(ratio[2], 6); Serial.print(", ");
  Serial.println(ratio[3], 6);
  Serial.println();
}

void runSensor(uint16_t table[11][4], bool raw){
  uint16_t rawR, rawG, rawB, rawC;
  float rVal, gVal, bVal, cVal;
  long avgR = 0, avgG = 0, avgB = 0, avgC = 0;
  
  for(int i = 0; i < 10; i++){
    colorSensor.getRawData(&rawR, &rawG, &rawB, &rawC);
    
    rVal = ((float) rawR / rawC) * 255;
    gVal = ((float) rawG / rawC) * 255;
    bVal = ((float) rawB / rawC) * 255;
    cVal = rawC;

    

    if(raw == false){
      calcOffset(&rVal, &gVal, &bVal, &cVal);
      calcSaturation(&rVal, &gVal, &bVal, &cVal);
      //calcRGB(&rVal, &gVal, &bVal, &cVal);
    }

    table[i][0] = rVal;
    table[i][1] = gVal;
    table[i][2] = bVal;
    table[i][3] = rawC;
    
    avgR += table[i][0];
    avgG += table[i][1];
    avgB += table[i][2];
    avgC += table[i][3];
  }
  
  table[10][0] = avgR / 10;
  table[10][1] = avgG / 10;
  table[10][2] = avgB / 10;
  table[10][3] = avgC / 10;
}

/*
  Calculates the offset for the rgbc values
*/
void calcOffset(float* r, float* g, float* b, float* c){
  float rOffset = ratio[0] * offset[0];
  float gOffset = ratio[1] * offset[1];
  float bOffset = ratio[2] * offset[2];

  float rMulti = (1 - ratio[0]) * multiplier[0];
  float gMulti = (1 - ratio[1]) * multiplier[1];
  float bMulti = (1 - ratio[2]) * multiplier[2];
  
  *r = (*r - rOffset) * (*r > rOffset);
  *g = (*g - gOffset) * (*g > gOffset);
  *b = (*b - bOffset) * (*b > bOffset);

  *r *= rMulti;
  *g *= gMulti;
  *b *= bMulti;
  
  *c = ((*c - offset[3]) * (offset[3] < *c)) + (offset[3] >= *c);
}

/*
  Caluculates the Saturation Values for the rgb from the clear value.
  Forumla: y = 0.7128x^(-0.757)
           saturated_rgb = rgb * c * y
*/
void calcSaturation(float* r, float* g, float* b, float* c){
  *c *= .7128 * pow(*c, multiplier[3]);  //-.757
  *r = *r * *c;
  *g = *g * *c;
  *b = *b * *c;
}

void printTable(char* title, uint16_t table[11][4]){
  char temp[100] = "";
  
  Serial.println(title);

  sprintf(temp, "|%-11s|%-11s|%-11s|%-11s|%-11s|", "", "Red", "Green", "Blue", "Clear");
  Serial.println(temp);

  for(int i = 0; i < 10; i++){
    sprintf(temp, "|%-11i|%-11u|%-11u|%-11u|%-11u|", i, table[i][0], table[i][1], table[i][2], table[i][3]);
    Serial.println(temp);
  }
  
  sprintf(temp, "|%-11s|%-11u|%-11u|%-11u|%-11u|", "Average", table[10][0], table[10][1], table[10][2], table[10][3]);
  Serial.println(temp);
}
