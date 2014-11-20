
///////////////////////////////////////
// Arduino 1.0.6
// ATMega 1284P (4K Bytes EEPROM)
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
#include <EEPROMAnything.h>
#include <Sunrise.h>
#include <I2C_eeprom.h>

int y_volts = 15;
int y_pres = 15;

#define DEBUG 1

#define UTC 3 //  UTC+3 = Moscow

struct config_t
{    
    unsigned long Display;
    int Contrast;
  
} configuration;

//---------------- IR Кнопки --------------------------

#define DISPLAY_1 16724175 // 1 Analog Clock
#define DISPLAY_2 16718055 // 2 Draw
#define DISPLAY_3 16743045 // 3 Date,Battary,Barametr and etc.
#define DISPLAY_4 16716015 // 4 GPS Info
#define DISPLAY_5 16726215 // 5 SunRise and SunSet
#define DISPLAY_6 16734885 // 6 Voltmetr
#define DISPLAY_7 16728765 // 7
#define DISPLAY_8 16730805 // 8
#define DISPLAY_9 16732845 // 9  GPS Output if BT is connected

// Для другово пульта кнопки

#define DIS_1 14614783
#define DIS_2 14647423
#define DIS_3 14631103
#define DIS_4 14663743
#define DIS_5 14622943
#define DIS_6 14655583
#define DIS_7 14639263
#define DIS_8 14671903
#define DIS_9 14618863

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

#define EEPROM_ADDRESS_256      (0x51) // 24LC256
#define EEPROM_ADDRESS_32       (0x50) // EEPROM on RTC 

#define BMP085_ADDRESS  0x77 // BMP085
#define DS1307_ADDRESS  0x68 // DS1307

#define EE24LC32MAXBYTES  32768/8
#define EE24LC256MAXBYTES 262144/8

I2C_eeprom eeprom32(EEPROM_ADDRESS_32  ,EE24LC32MAXBYTES);
I2C_eeprom eeprom256(EEPROM_ADDRESS_256,EE24LC256MAXBYTES);

// ee.writeByte(Address, Data); readByte(Address);

#define ONE_WIRE_BUS 20  // D20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress RTC_Thermometer = { 0x28, 0x46, 0xBD, 0x19, 0x3, 0x0, 0x0, 0x35 };

#define TEMPERATURE_PRECISION 12
#define ds oneWire

LCDShield lcd;  // Creates an LCDShield, named lcd

int CPU_LED = 1; // PB1 on Board

#define RECV_PIN 3 // D3

IRrecv irrecv(RECV_PIN);
decode_results results;

BMP085 dps = BMP085();    

long Temperature = 0, Pressure = 0, Altitude = 0;

SoftwareSerial bt(23,22); // RX,TX  

#define BT_CONNECT 30     // PA6/D30 HIGH if BT connected

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
unsigned long PreviousInterval = 0;     // Для всех внутренних функций
unsigned long loopPreviousInterval = 0; // Для управления GPS SetDateTime
unsigned long voltPreviousInterval = 0;  // Для вольтметра

int Contrast = 44;

boolean start = true; // Если была перегрузка.

void setup() {
  
  Wire.begin();  // Attach I2C Protocol
  delay(500);
    
  EEPROM_readAnything(0, configuration); // Чтения конфигурации
  
  Display = configuration.Display;   // Default DISPLAY_1;
  Contrast = configuration.Contrast; // Default 44
    
  set_1HZ_DS1307(); // Включаем синий светодиод на DS1307
  
  // delay(1000); // For BMP085 - Зачем не понятно  
  // setDateTime(); // Установка начального времени
 
  // dps.init(); == dps.init(MODE_STANDARD, 0, true); 
  // dps.init(MODE_STANDARD, 101850, false);
  
  pinMode(BT_CONNECT,INPUT);
  
  dps.init(MODE_ULTRA_HIGHRES, 25000, true);  // Разрешение BMP180
  
  irrecv.enableIRIn();
   
  pinMode(CPU_LED,OUTPUT);
  digitalWrite(CPU_LED,LOW);

  lcd.init(EPSON);   // Initializes lcd, using an PHILIPSdriver
  lcd.contrast(Contrast);  // -51's usually a good contrast value
  lcd.clear(BLACK);  // clear the screen

  Serial1.begin(4800);  // GPS EM-406
  bt.begin(9600);       // Bluetooth

  sensors.begin();
  sensors.setResolution(RTC_Thermometer, TEMPERATURE_PRECISION);


  eeprom32.writeByte(0,'a');
  bt.println(eeprom32.readByte(0))
  
  eeprom256.writeByte(0,'b');
  bt.println(eeprom256.readByte(0))
  
}

///////////////////////////////////////////////////////////////////////
//  MAIN LOOP 
///////////////////////////////////////////////////////////////////////

void loop() {
  
   currentMillis = millis();

   if (Display == DISPLAY_1) Analog_Time_Clock();
   if (Display == DISPLAY_2) Draw();
   if (Display == DISPLAY_3) ShowData(start);
   if (Display == DISPLAY_4) ShowDataGPS(start);
   if (Display == DISPLAY_5) ShowDataSun(start);
   if (Display == DISPLAY_6) ShowDataVolt(start);
   if (Display == DISPLAY_8) ShowBMP085(start);
   
   start = false; // Если была перегрузка
  
   if (irrecv.decode(&results)) {
    
     if (DEBUG) bt.println(results.value);
     
     digitalWrite(CPU_LED,HIGH);
     digitalWrite(CPU_LED,LOW);
     
    switch (results.value) {      
     case DIS_1: results.value = DISPLAY_1; break;
     case DIS_2: results.value = DISPLAY_2; break;
     case DIS_3: results.value = DISPLAY_3; break;
     case DIS_4: results.value = DISPLAY_4; break;
     case DIS_5: results.value = DISPLAY_5; break;
     case DIS_6: results.value = DISPLAY_6; break;
     case DIS_7: results.value = DISPLAY_7; break;
     case DIS_8: results.value = DISPLAY_8; break;
     case DIS_9: results.value = DISPLAY_9; break;
    }
     
    switch (results.value) {      
     case DISPLAY_1:
      Display = DISPLAY_1;
      lcd.clear(BLACK);
      configuration.Display = DISPLAY_1;
      EEPROM_writeAnything(0, configuration);
      break;
     case DISPLAY_2:
      Display = DISPLAY_2;
      lcd.clear(BLACK);
      configuration.Display = DISPLAY_2;
      EEPROM_writeAnything(0, configuration);
      break;
     case DISPLAY_3:
      Display = DISPLAY_3;
      configuration.Display = DISPLAY_3;
      EEPROM_writeAnything(0, configuration);
      lcd.clear(BLACK);
      ShowData(true);
      break;
      
     case DISPLAY_4:
      Display = DISPLAY_4;
      configuration.Display = DISPLAY_4;
      EEPROM_writeAnything(0, configuration);
      lcd.clear(BLACK);
      ShowDataGPS(true);
      break;
      
     case DISPLAY_5:
      Display = DISPLAY_5;
      configuration.Display = DISPLAY_5;
      EEPROM_writeAnything(0, configuration);
      lcd.clear(BLACK);
      ShowDataSun(true);
      break;
      
     case DISPLAY_6:
      Display = DISPLAY_6;
      configuration.Display = DISPLAY_6;
      EEPROM_writeAnything(0, configuration);
      lcd.clear(BLACK);
      ShowDataVolt(true);
      break;
      
     case DISPLAY_8:
      Display = DISPLAY_8;
      configuration.Display = DISPLAY_8;
      EEPROM_writeAnything(0, configuration);
      lcd.clear(BLACK);
      ShowBMP085(true);
      break;
      
     case CONTRAST_DW:
      if (Contrast < 60) Contrast++;  else Contrast=44;      
      lcd.contrast(Contrast);
      configuration.Contrast = Contrast;
      EEPROM_writeAnything(0, configuration);
      bt.println(Contrast);
      break;
      
     case CONTRAST_UP:
      if (Contrast > 20) Contrast--; else Contrast=44;     
      lcd.contrast(Contrast);
      configuration.Contrast = Contrast;
      EEPROM_writeAnything(0, configuration);
      bt.println(Contrast);
      break;
      
      
     case DISPLAY_9:
      if (GPS_OUT) GPS_OUT = false; else GPS_OUT = true;
      break;
      
    }
    
    irrecv.resume(); 
   }     


 // --------------------------- GPS -----------------------
 
  if(currentMillis - loopPreviousInterval > 5123) { 
   loopPreviousInterval = currentMillis;  
   if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid())
     set_GPS_DateTime();
  }
   
   if (Serial1.available()) {
     char nmea = Serial1.read();
     gps.encode(nmea);
     if (GPS_OUT && (digitalRead(BT_CONNECT) == HIGH)) bt.print(nmea);  
   }
}


// --------------------------- Мигает светодиод 1 HZ от RTC DS1307 -------------------------------

void set_1HZ_DS1307( void ) {
    
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x07);
  Wire.write(0x10);              // Set Square Wave to 1 Hz
  Wire.endTransmission();

}

// -------------------------- Измерение входного напряжения от батарейки 3.7V Батарейка 4.x - Зарядка ----

float battary( void ) {
  
  int sensorValue = analogRead(A4); // PA4/D28
  float voltage = sensorValue * (5.0 / 1023.0);
  return(voltage);
  
}

// --------------------------------------------- Функция для Аналоговых часов --------------------

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

// ---------------------------- Установка времени через GPS ------------------------

void set_GPS_DateTime() {
  
 if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {

  byte seconds =   gps.time.second();
  byte minutes =   gps.time.minute();
  byte hours =     gps.time.hour();
  
  hours = hours + UTC;
  if (hours > 23)  hours = hours - 24;
  
  byte weekDay =   1;
  byte monthDay =  gps.date.day();
  byte months =    gps.date.month();
  byte years  =    gps.date.year() - 2000;

  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0);

  Wire.write(decToBcd(seconds));
  Wire.write(decToBcd(minutes));
  Wire.write(decToBcd(hours));
  Wire.write(decToBcd(weekDay));
  Wire.write(decToBcd(monthDay));
  Wire.write(decToBcd(months));
  Wire.write(decToBcd(years));

  Wire.write(0); 
  Wire.endTransmission();
}

}

// ---------------------------------- Отображаем аналоговые часы -----------------------------

void Analog_Time_Clock( void ) {
    
  if(currentMillis - PreviousInterval > 1000) { 
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



// ----------------------------------- getTemperature (no delay) -------------------

float f2c(float val){
  float aux = val - 32;
  return (aux * 5 / 9);
}

void writeTimeToScratchpad(byte* address){
  ds.reset();
  ds.select(address);
  ds.write(0x44,1);
  delay(10);
}

void readTimeFromScratchpad(byte* address, byte* data){
  ds.reset();
  ds.select(address);
  ds.write(0xBE);
  for (byte i=0;i<9;i++){
    data[i] = ds.read();
  }
}

float getTemperature(byte* address){

  int tr;
  byte data[12];

  writeTimeToScratchpad(address);
  readTimeFromScratchpad(address,data);

  tr = data[0];

  if (data[1] > 0x80){
    tr = !tr + 1; 
    tr = tr * -1; 
  }

  int cpc = data[7];
  int cr = data[6];

  tr = tr >> 1;

  return tr - (float)0.25 + (cpc - cr)/(float)cpc;
}

// -------------------------------- Show Data on Display 3 -----------------------------------

void ShowData(boolean s) {

   char output[20];
   char f[30];
   
  if ((currentMillis - PreviousInterval > 1000) || (s == true) ) { 
   PreviousInterval = currentMillis;  
      
   dps.getPressure(&Pressure); 
   dps.getAltitude(&Altitude); 
   dps.getTemperature(&Temperature);

   dtostrf(Altitude/100.0, 4, 2, output);
   strcpy(f,"Alt: ");
   strcat(f,output);
   lcd.setStr(f,0,2,WHITE, BLACK);
    
   dtostrf(Pressure/133.3, 4, 2, output);
   strcpy(f,"Press: ");
   strcat(f,output);
   lcd.setStr(f,15,2,WHITE, BLACK);

   dtostrf(Temperature*0.1, 4, 2, output);
   strcpy(f,"T[in]: ");
   strcat(f,output);
   lcd.setStr(f,30,2,WHITE, BLACK);
       
   dtostrf(battary(), 4, 2, output);
   strcpy(f,"Vin: ");
   strcat(f,output);
   lcd.setStr(f,45,2,WHITE, BLACK);       
   
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
   
   
   sprintf(output, "%.2d:%.2d:%.2d", hours, minutes, seconds);
   strcpy(f,"Time: ");
   strcat(f,output);
   lcd.setStr(f,60,2,WHITE, BLACK);       

   sprintf(output, "%.2d/%.2d/%.2d", monthDay, month, year);
   strcpy(f,"Date: ");
   strcat(f,output);
   lcd.setStr(f,75,2,WHITE, BLACK);       
     
   float tempC = getTemperature(RTC_Thermometer);
         tempC = f2c(tempC);
   
   dtostrf(tempC, 4, 2, output);
   strcpy(f,"T[rtc]: ");
   strcat(f,output);
   lcd.setStr(f,90,2,WHITE, BLACK);  

   if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    strcpy(f,"GPS: OK  ");
    lcd.setStr(f,105,2,WHITE, BLACK);  
   } else {
    strcpy(f,"GPS: None");
    lcd.setStr(f,105,2,WHITE, BLACK);  
   }
       
   
  }
}

void Draw( void ) {
  
 int segmentsRed[1] = {ENE};
 lcd.setArc(60,50,40,segmentsRed,sizeof(segmentsRed),5,RED);
 int segmentsYellow[1] = {NNE};
 lcd.setArc(60,50,40,segmentsYellow,sizeof(segmentsYellow),10,YELLOW);
  
 int segments[4] = {WNW,NNW,NNE,ENE};
  
 lcd.setArc(60,50,40,segments,sizeof(segments),FILL,YELLOW);
 delay(100);
 lcd.setArc(60,50,40,segments,sizeof(segments),FILL,BLACK);
 delay(100);
   
 lcd.setCircle(90,100,20,PINK,FILL);
 lcd.setCircle(90,35,25,CYAN,3);
 Display = DISPLAY_NONE;

}

// -------------------------------- Show Data on Display 4 GPS/BT -----------------------------------

void ShowDataGPS(boolean s) {

   char output[20];
   char f[30];
   
  if ((currentMillis - PreviousInterval > 1000) || (s == true) ) { 
   PreviousInterval = currentMillis;  
      

   if (digitalRead(BT_CONNECT) == HIGH) strcpy(f,"BT: Connect"); else
                                        strcpy(f,"BT: None   "); 
   
   lcd.setStr(f,0,2,WHITE, BLACK);
   
   
    if (gps.location.isValid()) {
      
         dtostrf(gps.location.lat(),4,6, output);
         strcpy(f,"N: ");
         strcat(f,output);
         lcd.setStr(f,15,2,WHITE, BLACK);  

         dtostrf(gps.location.lng(),4,6, output);
         strcpy(f,"E: ");
         strcat(f,output);
         lcd.setStr(f,30,2,WHITE, BLACK);  

    } else {

      strcpy(f,"N: No GPS   ");
      lcd.setStr(f,15,2,WHITE, BLACK);  

      strcpy(f,"E: No GPS   ");
      lcd.setStr(f,30,2,WHITE, BLACK);        
    }
    

    if (gps.date.isValid() && gps.time.isValid()) {
            
      sprintf(f,"gTime:%.2d:%.2d:%.2d",gps.time.hour()+UTC,gps.time.minute(),gps.time.second());
      lcd.setStr(f,45,2,WHITE, BLACK);              
      sprintf(f,"gDate:%.2d/%.2d/%.2d",gps.date.day(),gps.date.month(),gps.date.year());
      lcd.setStr(f,60,2,WHITE, BLACK);        

    } else {
      
      sprintf(f,"gTime: No GPS   ");
      lcd.setStr(f,45,2,WHITE, BLACK);              
      sprintf(f,"gDate: No GPS   ");
      lcd.setStr(f,60,2,WHITE, BLACK);        
    }

         dtostrf(gps.speed.kmph(),4,2, output);
         strcpy(f,"Speed: ");
         strcat(f,output);
         lcd.setStr(f,75,2,WHITE, BLACK);  

         dtostrf(gps.satellites.value(),4,2, output);
         strcpy(f,"Satt: ");
         strcat(f,output);
         lcd.setStr(f,90,2,WHITE, BLACK);  

         dtostrf(gps.hdop.value(),4,2, output);
         strcpy(f,"HDOP: ");
         strcat(f,output);
         lcd.setStr(f,105,2,WHITE, BLACK);     
   
  }
  
}


// ---------------- Sun Rise and Sun Set -------------------------------

void ShowDataSun( boolean s) { 
  
  byte m_set=0,h_set=0;
  byte m_rise=0,d,h_rise=0;
  int t;   
  int segments[4] = {WNW,NNW,NNE,ENE};  
  char f[30];
   
  if ((currentMillis - PreviousInterval > 1000) || (s == true) ) { 
   PreviousInterval = currentMillis;  

   strcpy(f, "Sun:");
   lcd.setStr(f,0,2,YELLOW,BLACK);   
  
  if (gps.location.isValid()) {
      
   Sunrise mySunrise(gps.location.lat(),gps.location.lng(),UTC);
   mySunrise.Actual();

   t = mySunrise.Rise(11,19); // Month, Day
   if(t >= 0) {
    h_rise = mySunrise.Hour();
    m_rise = mySunrise.Minute();
   } else { h_rise = 0; m_rise = 0; }    
   
   t = mySunrise.Set(11,19); // Month, Day
   if(t >= 0) {
    h_set = mySunrise.Hour();
    m_set = mySunrise.Minute();  
   } else { h_set = 0; m_set = 0; }
   
  }

   sprintf(f, "Sun Rise: %.2d:%.2d",h_rise,m_rise);
   lcd.setStr(f,15+2,2,WHITE, BLACK);   
   
   sprintf(f, "Sun Set:  %.2d:%.2d",h_set,m_set);
   lcd.setStr(f,30+2,2,WHITE, BLACK);         

   strcpy(f, "Real Time:");
   lcd.setStr(f,45+4,2,YELLOW,BLACK);   
  
  if (gps.date.isValid() && gps.time.isValid()) {
            
      sprintf(f,"gTime:%.2d:%.2d:%.2d",gps.time.hour()+UTC,gps.time.minute(),gps.time.second());
      lcd.setStr(f,60+7,2,WHITE, BLACK);              
      sprintf(f,"gDate:%.2d/%.2d/%.2d",gps.date.day(),gps.date.month(),gps.date.year());
      lcd.setStr(f,75+7,2,WHITE, BLACK);        

    } else {
      
      sprintf(f,"gTime: No GPS   ");
      lcd.setStr(f,60+7,2,WHITE, BLACK);              
      sprintf(f,"gDate: No GPS   ");
      lcd.setStr(f,75+7,2,WHITE, BLACK);        
    }

}
 
   // lcd.setArc(131,65,30,segments,sizeof(segments),FILL,WHITE);   
   // lcd.setArc(131,65,30,segments,sizeof(segments),FILL,BLACK);
   
}
// ------------------------------- Вольтметр -----------------------------

void ShowDataVolt(boolean s) {

  if ((currentMillis - voltPreviousInterval > 30000) || (s == true) ) {  // 900 000
   voltPreviousInterval = currentMillis;  
  
  // x,y x,y
  
  lcd.setLine(1,14,105,14,WHITE);
  lcd.setLine(107,0,107,129,WHITE);

  // Вольты  
  lcd.setChar('6',  15,12, WHITE,BLACK);
  lcd.setChar('5',  30,12, WHITE,BLACK);
  lcd.setChar('4',  45,12, WHITE,BLACK);
  lcd.setChar('3',  60,12, WHITE,BLACK);
  lcd.setChar('2',  75,12, WHITE,BLACK);
  lcd.setChar('1',  90,12, WHITE,BLACK);
  lcd.setChar('0', 105,12, WHITE,BLACK);
  
  // Время
  lcd.setChar('0', 122,21, WHITE,BLACK);
  lcd.setChar('4', 122,32, RED,BLACK);
  lcd.setChar('8', 122,42+1, WHITE,BLACK);
  
  lcd.setChar('1', 122,52+1, RED,BLACK);
  lcd.setChar('2', 122,62+1, RED,BLACK);
  
  lcd.setChar('1', 122,72+2, WHITE,BLACK);
  lcd.setChar('6', 122,82+2, WHITE,BLACK);
  
  lcd.setChar('2', 122,92+4,  RED,BLACK);
  lcd.setChar('0', 122,102+4, RED,BLACK);
  
  lcd.setChar('2', 122,112+6, WHITE,BLACK);
  lcd.setChar('4', 122,122+6, WHITE,BLACK);
  
  // lcd.setPixel(WHITE, 106,15);

  // for(int y=15;y<130;y++) {
  // int x=map(random(0,6),0,6,106,0);
  //  lcd.setLine(x,y,106,y, WHITE);
  // }

  int x = map(battary(),0.0,5.0,106,0);
  lcd.setLine(x,y_volts,106,y_volts, WHITE);
  y_volts++; if (y_volts > 130) y_volts=15;
  
  }  
  
}

// ---------------- Barometer Graphics ------------------------

void ShowBMP085(boolean s) {

  if ((currentMillis - PreviousInterval > 30000) || (s == true) ) {  // 900 000
   PreviousInterval = currentMillis;  
  
  dps.getPressure(&Pressure);  // Get data from BMP085

  // x,y x,y две линии по X,Y
  
  lcd.setLine(1,14,105,14,WHITE);
  lcd.setLine(107,0,107,129,WHITE);

  // Давление  

  lcd.setChar('B',  15,12, WHITE,BLACK);
  lcd.setChar('A',  30,12, WHITE,BLACK);
  lcd.setChar('R',  45,12, WHITE,BLACK);
  lcd.setChar('O',  60,12, WHITE,BLACK);
  lcd.setChar('M',  75,12, WHITE,BLACK);
  lcd.setChar('T',  90,12, WHITE,BLACK);
  lcd.setChar('R', 105,12, WHITE,BLACK);
  
  // Время
  lcd.setChar('0', 122,21, WHITE,BLACK);
  lcd.setChar('4', 122,32, RED,BLACK);
  lcd.setChar('8', 122,42+1, WHITE,BLACK);
  
  lcd.setChar('1', 122,52+1, RED,BLACK);
  lcd.setChar('2', 122,62+1, RED,BLACK);
  
  lcd.setChar('1', 122,72+2, WHITE,BLACK);
  lcd.setChar('6', 122,82+2, WHITE,BLACK);
  
  lcd.setChar('2', 122,92+4,  RED,BLACK);
  lcd.setChar('0', 122,102+4, RED,BLACK);
  
  lcd.setChar('2', 122,112+6, WHITE,BLACK);
  lcd.setChar('4', 122,122+6, WHITE,BLACK);
  
  // lcd.setPixel(WHITE, 106,15);

  // for(int y=15;y<130;y++) {
  // int x=map(random(0,6),0,6,106,0);
  //  lcd.setLine(x,y,106,y, WHITE);
  // }

  int x = map((Pressure/133.3),600,900,106,0);
  lcd.setLine(x,y_pres,106,y_pres, WHITE);
  y_pres++; if (y_pres > 130) y_pres=15;
 
  
  }  
  
}


