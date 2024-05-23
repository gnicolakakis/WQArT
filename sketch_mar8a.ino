#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "GravityTDS.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "DFRobot_RGBLCD1602.h"

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 6
// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);	
// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

 
#define VREF    5000//VREF(mv)
#define ADC_RES 1024//ADC Resolution
 
uint32_t raw;

 
#define TdsSensorPin A2
GravityTDS gravityTds;
 
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10],temp;


float temperature = 25,tdsValue = 0;
 


//variable for SD card 
File myFile;


//lcd initialization 
const int colorR = 255;
const int colorG = 255;
const int colorB = 255;

DFRobot_RGBLCD1602 lcd(/*RGBAddr*/0x60 ,/*lcdCols*/16,/*lcdRows*/2);  //16 characters and 2 lines of show



//DOS initialization
#define DO_PIN A7
#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution
 
//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0
 
#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function
 
//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1704) //mv
#define CAL1_T (22)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
//#define CAL2_V (1690) //mv
//#define CAL2_T (15)   //℃
 
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};
 
uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;
 
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 00
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

void setup(void){
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("SD initialization done.");
  sensors.begin();	// Start up the library
  
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization

  lcd.init();
    
    lcd.setRGB(colorR, colorG, colorB);
    
    // Print starting messege to the LCD.
    lcd.print(" WATER QUALITY");
    lcd.setCursor(0,1);
    lcd.print("     CHECK");
    

    delay(1000);
 
}

void loop(void)
{ 
  // Send the command to get temperatures
  sensors.requestTemperatures(); 

  //print the temperature in Celsius
  Serial.print("Temperature: ");
  Serial.print(sensors.getTempCByIndex(0));
  Serial.print((char)176);//shows degrees character
  Serial.println("C");
  
  
  delay(1000);

    //temperature = readTemperature();  //add your temperature sensor and read it
    gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    gravityTds.update();  //sample and calculate
    tdsValue = gravityTds.getTdsValue();  // then get the value
    Serial.print("TDS (ppm)"); 
    Serial.println(tdsValue,0);
    
    delay(1000);

    
//turbidity sensor
  int sensorValue2 = analogRead(A0);
  float turbidityvar = sensorValue2 * (5.0 / 1024.0);
 
  Serial.print("Sensor Output (turbidity sensor) (V):");
  Serial.print(turbidityvar);
  Serial.println();
  delay(1000);

  
//ph
for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(A4);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue;                      //convert the millivolt into pH value
  Serial.print("pH:");  
  Serial.print(phValue,2);
  Serial.println(" ");
 
  delay(1000);



  //dos

  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
 
 //logika axristo Serial.print("Temperaturet:\t" + String(Temperaturet) + "\t");
  Serial.print("ADC RAW:\t" + String(ADC_Raw) + "\t");
  Serial.print("ADC Voltage:\t" + String(ADC_Voltage) + "\t");
  Serial.println("DO:\t" + String(readDO(ADC_Voltage, Temperaturet)) + "\t");
  delay(1000);
  Serial.println();
  Serial.println();

 
  
  // if the file opened okay, write to it:
  

   myFile = SD.open("arduino.csv", FILE_WRITE);

  if (myFile) {
    //temp
    myFile.println(sensors.getTempCByIndex(0));
    myFile.print(",");

    //tds
    myFile.println(tdsValue,0);
    myFile.print(",");
    
    
    //turbidity
    myFile.println(turbidityvar);
    myFile.print(",");

    //ph
    myFile.println(phValue,2);
    myFile.print(",");
    
    //dos
    
    myFile.println("DO:\t" + String(readDO(ADC_Voltage, Temperaturet)) + "\t");
    myFile.print(",");
    myFile.println();
    
    myFile.close();
  } else {
    Serial.print(F("SD Card: error on opening file arduino.txt"));
  }
  lcd.clear();
  if (tdsValue<=220 && voltage>4) {
    lcd.setRGB(0,255,0);		      // good quality water Green
    lcd.setCursor(0,0);  			//First line
    lcd.print("Good Quality");
    lcd.setCursor(0,1); 			//Second line
    lcd.print("Drinkable");
  }
  else if (tdsValue<=400 && voltage>3.5) {
    lcd.setRGB(240,135,8);		// medium quality water Blue
    lcd.setCursor(0,0); 			//First line
    lcd.print("Medium Quality");
    lcd.setCursor(0,1); 			//Second line
    lcd.print("Decide if drink");
  }
  else {
    lcd.setRGB(255,0,0);			// bad quality water Red
    lcd.setCursor(0,0); 			//First line
    lcd.print("Bad Quality");
    lcd.setCursor(0,1); 			//Second line
    lcd.print("Not Drinkable");
  }

}