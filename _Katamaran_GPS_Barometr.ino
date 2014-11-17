///////////////////////////////////////
// Arduino 1.0.6
///////////////////////////////////////

#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <IRremote.h>
#include <Wire.h>
#include <ColorLCDShield.h>
#include <BMP085.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"

#define DEBUG 1

struct config_t
{
    long alarm;
    int mode;

} configuration;


//---------------- IR Кнопки --------------------------

#define DISPLAY_1 16724175 // 1 Analog Clock
#define DISPLAY_2 16718055 // 2 Draw
#define DISPLAY_3 16743045 // 3
#define DISPLAY_4 16716015 // 4
#define DISPLAY_5 16726215 // 5
#define DISPLAY_6 16734885 // 6
#define DISPLAY_7 16728765 // 7
#define DISPLAY_8 16730805 // 8
#define DISPLAY_9 16732845 // 9  GPS Output

#define CONTRAST_UP 16769055 // +
#define CONTRAST_DW 16754775 // -

#define DISPLAY_NONE 0

// BOX G218C Chip-Dip

////////////////////////////////////////////
// PA4/D28/ AnalogInput Battary Voltage  //
// GPS PPS PD5/D13                       //
///////////////////////////////////////////

// http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html
// http://www.pjrc.com/teensy/td_libs_OneWire.html
// http://bigbarrel.ru/eeprom/

#define EEPROM_ADDRESS      0x51 // 24LC256
#define EEPROM_ADDRESS_RTC  0x50 // EEPROM on RTC

#define BMP085_ADDRESS  0x77 // BMP085
#define DS1307_ADDRESS  0x68 // DS1307

#define ONE_WIRE_BUS 20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress RTC_Thermometer = { 0x28, 0x46, 0xBD, 0x19, 0x3, 0x0, 0x0, 0x35 };

LCDShield lcd;  // Creates an LCDShield, named lcd

int CPU_LED = 1; // PB1 on Board

#define RECV_PIN 3 // D3

IRrecv irrecv(RECV_PIN);
decode_results results;

BMP085 dps = BMP085();    

long Temperature = 0, Pressure = 0, Altitude = 0;

SoftwareSerial bt(23,22); // RX,TX  

TinyGPSPlus gps;
boolean GPS_OUT = false;

//////////////////////////////////////////// Часы ////////////////////////

// Enter the time below in 12-hr format

#define CLOCK_RADIUS 45  // radius of clock face
#define CLOCK_CENTER 50  // If you adjust the radius, you'll probably want to adjust this
#define H_LENGTH  25     // length of hour hand
#define M_LENGTH  35     // length of minute hand
#define S_LENGTH  43     // length of second hand

#define BACKGROUND  BLACK   // room for growth, adjust the background color according to daylight
#define C_COLOR     RED     // This is the color of the clock face, and digital clock
#define H_COLOR     BLUE    // hour hand color
#define M_COLOR     GREEN   // minute hand color
#define S_COLOR     YELLOW  // second hand color

int hours, minutes, seconds, ampm;
int weekDay,monthDay,month,year;

////////////////////////////////////////////

unsigned long Display;

unsigned long currentMillis;
unsigned long PreviousInterval = 0; 

void setup() {
  
  Display = DISPLAY_1;
  
  Wire.begin();  // Attach I2C Protocol
    
  EEPROM_readAnything(0, configuration); // Чтения конфигурации
  set_1HZ_DS1307(); // Включаем синий светодиод на DS1307
  
  // delay(1000); // For BMP085 - Зачем не понятно  
  // setDateTime(); // Установка начального времени
 
  // dps.init(); == dps.init(MODE_STANDARD, 0, true); 
  // dps.init(MODE_STANDARD, 101850, false);
  
  dps.init(MODE_ULTRA_HIGHRES, 25000, true);  // Разрешение BMP180
  
  irrecv.enableIRIn();
   
  pinMode(CPU_LED,OUTPUT);
  digitalWrite(CPU_LED,LOW);

  lcd.init(EPSON);   // Initializes lcd, using an PHILIPSdriver
  lcd.contrast(44);  // -51's usually a good contrast value
  lcd.clear(BLACK);  // clear the screen

  Serial1.begin(4800);  // GPS EM-406
  bt.begin(9600);       // Bluetooth

}

void loop()
{
  
  
   currentMillis = millis();

   if (Display == DISPLAY_1) Analog_Time_Clock();
   if (Display == DISPLAY_2) Draw();
   if (Display == DISPLAY_3) ShowData();
   
    // Test
    // i2scan();
  
   if (irrecv.decode(&results)) {
    
     if (DEBUG) bt.println(results.value);
     
     digitalWrite(CPU_LED,HIGH);
     delay(50);
     digitalWrite(CPU_LED,LOW);
     
    switch (results.value) {      
     case DISPLAY_1:
      Display = DISPLAY_1;
      lcd.clear(BLACK);
      break;
     case DISPLAY_2:
      Display = DISPLAY_2;
      lcd.clear(BLACK);
      break;
     case DISPLAY_3:
      Display = DISPLAY_3;
      lcd.clear(BLACK);
      break;
      
     case DISPLAY_9:
      if (GPS_OUT) GPS_OUT = false; else GPS_OUT = true;
      break;
      
    }
    
    irrecv.resume(); 
   }     


 // --------------------------- GPS -----------------------
 
   if (Serial1.available()) {
     char nmea = Serial1.read();
     gps.encode(nmea);
     if (GPS_OUT) bt.print(nmea);
  
  /*
    if (gps.location.isValid()) {
       bt.println(gps.location.lat(), 6);
       bt.println(gps.location.lng(), 6);     
     }   
         
     if (gps.date.isValid()) {
       bt.print(gps.date.month());
       bt.print(F("/"));
       bt.print(gps.date.day());
       bt.print(F("/"));
       bt.println(gps.date.year());
     }

     if (gps.time.isValid()) {
      bt.print(gps.time.hour());
      bt.print(F(":"));
      bt.print(gps.time.minute());
      bt.print(F(":"));
      bt.println(gps.time.second());
     }
   */
   
   }
   
 /*  
  dps.getPressure(&Pressure); 
  dps.getAltitude(&Altitude); 
  dps.getTemperature(&Temperature);

  bt.print("  Alt(m):"); 
  bt.print(Altitude/100.0); 
  bt.print("  Pressure(mm Hg):"); 
  bt.print(Pressure/133.3); 
  bt.print(" Temp:"); 
  bt.println(Temperature*0.1); 
  delay(2000); 
  
  */
  
}


void i2scan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}

/*


        void init(int type, bool colorSwap = 0);
	void clear(int color);
	void contrast(char setting);

	void setPixel(int color, unsigned char x, unsigned char y);
	void setCircle (int x0, int y0, int radius, int color, int lineThickness = 1);
	void setArc(int x0, int y0, int radius, int segments[], int numSegments, int lineThickness, int color);

	void setChar(char c, int x, int y, int fColor, int bColor);
	void setStr(char *pString, int x, int y, int fColor, int bColor);
	

	void setLine(int x0, int y0, int x1, int y1, int color);
	void setRect(int x0, int y0, int x1, int y1, unsigned char fill, int color);
    
        void printBMP(char image_main[2048]);

	void printLogo(void);

	void on(void);
	void off(void);

*/


void set_1HZ_DS1307( void ) {
    
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x07);
  Wire.write(0x10);  // Set Square Wave to 1 Hz
  Wire.endTransmission();

}

void battary( void ) {
  
  int sensorValue = analogRead(A4); // PA4/D28
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);
  // print out the value you re
  
}


void displayDigitalTime(int h, int m, int s)
{
    char timeChar[12];
  
    sprintf(timeChar, "%.2d:%.2d:%.2d", h, m, s);
    lcd.setStr(timeChar, CLOCK_CENTER + CLOCK_RADIUS + 10, 25, C_COLOR, BACKGROUND);
}

void drawClock()
{
  lcd.setCircle(CLOCK_CENTER, 66, CLOCK_RADIUS, C_COLOR);
  lcd.setStr("12", CLOCK_CENTER - CLOCK_RADIUS, 66-9, C_COLOR, BACKGROUND);
  lcd.setStr("3", CLOCK_CENTER - 9, 66 + CLOCK_RADIUS - 12, C_COLOR, BACKGROUND);
  lcd.setStr("6", CLOCK_CENTER + CLOCK_RADIUS - 18, 66-4, C_COLOR, BACKGROUND);
  lcd.setStr("9", CLOCK_CENTER - 9, 66 - CLOCK_RADIUS + 4, C_COLOR, BACKGROUND);
}

void displayAnalogTime(int h, int m, int s) {
  
  double midHours;  
  static int hx, hy, mx, my, sx, sy;
  
  h -= 3;
  m -= 15;
  s -= 15;
  
  if (h <= 0) h += 12;
  if (m < 0)  m += 60;
  if (s < 0)  s += 60;
    
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+sx, 66+sy, BACKGROUND);  // delete second hand
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+mx, 66+my, BACKGROUND);  // delete minute hand
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+hx, 66+hy, BACKGROUND);  // delete hour hand
  
  /* Calculate and draw new lines: */
  
  s = map(s, 0, 60, 0, 360);                                      // map the 0-60, to "360 degrees"
  sx = S_LENGTH * sin(3.14 * ((double) s)/180);                   // woo trig!
  sy = S_LENGTH * cos(3.14 * ((double) s)/180);                   // woo trig!
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+sx, 66+sy, S_COLOR); // print second hand
  
  m = map(m, 0, 60, 0, 360);                                      // map the 0-60, to "360 degrees"
  mx = M_LENGTH * sin(3.14 * ((double) m)/180);                   // woo trig!
  my = M_LENGTH * cos(3.14 * ((double) m)/180);                   // woo trig!
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+mx, 66+my, M_COLOR); // print minute hand
  
  midHours = minutes/12;  
  h *= 5;                                                         
  h += midHours;                                                  // add hours and midhours
  h = map(h, 0, 60, 0, 360);                                      // map the 0-60, to "360 degrees"
  hx = H_LENGTH * sin(3.14 * ((double) h)/180);                   // woo trig!
  hy = H_LENGTH * cos(3.14 * ((double) h)/180);                   // woo trig!
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+hx, 66+hy, H_COLOR); // print hour hand

}


////////////////////////////////////////// DS1307 /////////////////////////////////////////////

byte decToBcd(byte val) {
  return ( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val) {
  return ( (val/16*10) + (val%16) );
}

void setDateTime() {

  byte second =      00; //0-59
  byte minute =      00; //0-59
  byte hour =        12; //0-23
  byte weekDay =      1; //1-7
  byte monthDay =    17; //1-31
  byte month =       11; //1-12
  byte year  =       14; //0-99

  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0);

  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write(decToBcd(weekDay));
  Wire.write(decToBcd(monthDay));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));

  Wire.write(0); 
  Wire.endTransmission();

}

void Analog_Time_Clock( void ) {
    
   if(currentMillis - PreviousInterval > 1000) {  // Выводим большие часы
    PreviousInterval = currentMillis;  
  
   Wire.beginTransmission(DS1307_ADDRESS);
   Wire.write(0);
   Wire.endTransmission();
   Wire.requestFrom(DS1307_ADDRESS, 7);

   seconds = bcdToDec(Wire.read());
   minutes = bcdToDec(Wire.read());
   hours = bcdToDec(Wire.read() & 0b111111); //24 hour time
  
   weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
   monthDay = bcdToDec(Wire.read());
   month = bcdToDec(Wire.read());
   year = bcdToDec(Wire.read());

   drawClock();
   displayAnalogTime(hours, minutes, seconds);
   displayDigitalTime(hours, minutes, seconds);

  }
  
}

void ShowData( void ) {

   char output[20];
   int a = 10;
   
   
  if(currentMillis - PreviousInterval > 1000) {  // Выводим большие часы
   PreviousInterval = currentMillis;  
   lcd.clear(BLACK);
   
   sprintf(output,"%2d",a);
   lcd.setStr(output,0,0,WHITE, BLACK);
  }
}

void Draw( void ) {
  
   int segmentsRed[1] = {ENE};
   lcd.setArc(60,50,40,segmentsRed,sizeof(segmentsRed),5,RED);
   int segmentsYellow[1] = {NNE};
   lcd.setArc(60,50,40,segmentsYellow,sizeof(segmentsYellow),10,YELLOW);
   int segments[2] = {WNW,NNW};
   lcd.setArc(60,50,40,segments,sizeof(segments),FILL,GREEN);
   lcd.setCircle(90,100,20,PINK,FILL);
   lcd.setCircle(90,35,25,CYAN,3);
   Display = DISPLAY_NONE;

}
