
//*******************************************************
// Arduino 1.0.6                     
// ATMega 1284P (4K Bytes EEPROM)   
// Update: 25.09.2015                
//*******************************************************

// #define _SS_MAX_RX_BUFF 64 // RX buffer size => 256
// SoftwareSerial.h Изменить размер RX буфера.

// ESP 8266 Version

// AT version:0.22.0.0(Mar 20 2015 10:04:26)
// SDK version:1.0.0
// compile time:Mar 20 2015 11:00:32

//  AT+CWSAP_DEF="GPS","",6,0
//  AT+CWSAP_CUR="GPS","",6,0

//  AT+CWMODE=3  Soft_AP + Client
//  AT+CIPMUX=1 Много соединений
//  AT+CIPSERVER=1,80  Сервер на 80 порту

// https://github.com/itead/ITEADLIB_Arduino_WeeESP8266
// 
// #define USER_SEL_VERSION	VERSION_22
// #define ESP8266_USE_SOFTWARE_SERIAL


  //  0.0
  //   --------------------------> Y
  //   |
  //   |
  //   |
  //   v
  //   X
  
#include <ESP8266.h>
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
#include <RTClib.h>
#include <Average.h>

#define DEBUG 0

RTC_DS1307 rtc;  // DS1307 RTC Real Time Clock

#define FIVE_MINUT 300000
#define TWO_DAYS 172800

// ------------------------------

int y_volts = 15;

byte X_Menu = 1;  // Keyboard for SetupMenu

unsigned long BAR_EEPROM_POS = 0;

// -------------------------------

#define UTC 3 //  UTC+3 = Moscow

struct config_t
{    
    unsigned long Display;
    int Contrast;
    unsigned long Last_GPS_Pos; // Последняя точка записи в EEPROM GPS Координат
    boolean GPS_OUT;
  
} configuration;

// --------------------------------------------------------------------

struct gps_t   // Координаты для GPS Трекера sizeof == 14 Byte 
{
  int years;
  byte days,months,hours,minutes;
  double lats,lngs;
  
} gps_tracker;

// ----------------------- BMP085 ---------------------------------

struct bmp085_t // Данные о давлении,высоте и температуре
{    
    double Press,Alt,Temp;
    unsigned long unix_time; 
    
} bmp085_data;

struct bmp085_out // Данные о давлении,высоте и температуре
{    
    double Press,Alt,Temp;
    unsigned long unix_time; 
    
} bmp085_data_out;

//---------------- IR Кнопки --------------------------

#define DISPLAY_1 16724175 // 1 Analog Clock
#define DISPLAY_2 16718055 // 2 SetupMenu()
#define DISPLAY_3 16743045 // 3 Date,Battary,Barametr and etc.
#define DISPLAY_4 16716015 // 4 GPS Info
#define DISPLAY_5 16726215 // 5 SunRise and SunSet
#define DISPLAY_6 16734885 // 6 Voltmetr
#define DISPLAY_7 16728765 // 7 GPS_Track Output
#define DISPLAY_8 16730805 // 8 Barometer
#define DISPLAY_9 16732845 // 9 GPS Output if BT is connected

#define DISPLAY_10 10000001 // Температура
#define DISPLAY_11 10000002 // Алтиметр

#define DOWN   1
#define UP     2
#define LEFT   3
#define RIGHT  4
#define ENTER  5
#define ONOFF  6

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

#define DIS_ONOFF 14627023
#define DIS_UP    14616823
#define DIS_DOWN  14649463
#define DIS_LEFT  14633143
#define DIS_RIGHT 14665783
#define DIS_ENTER 14643343

#define DI_1 16582903
#define DI_2 16615543
#define DI_3 16599223
#define DI_4 16591063
#define DI_5 16623703
#define DI_6 16607383
#define DI_7 16586983
#define DI_8 16619623
#define DI_9 16603303

#define DI_ONOFF  16580863
#define DI_UP     16613503 // VOL+
#define DI_DOWN   16617583 // VOL-
#define DI_LEFT   16605343 // >>|| Reverse Right
#define DI_RIGHT  16589023 // ||<< Reverse Left
#define DI_ENTER  16621663 // >|| Play

#define CONTRAST_UP 16769055 // +
#define CONTRAST_DW 16754775 // -

#define DISPLAY_NONE 0 // Не обновляем экран

#define DISPLAY_MENU 1 // Если включен режим SetupMenu()

#define MAX_MENU     13 // Всего Меню на экране 1-MAX_MENU
#define MAX_DISPLAY  8  // Максимальое количество меню на экране

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

#define EE24LC32MAXBYTES   32768/8
#define EE24LC256MAXBYTES 262144/8 // 32768 Байт [0-32767]

I2C_eeprom  eeprom32(EEPROM_ADDRESS_32 ,EE24LC32MAXBYTES);
I2C_eeprom eeprom256(EEPROM_ADDRESS_256,EE24LC256MAXBYTES);

// ee.writeByte(Address, Data); readByte(Address);

#define ONE_WIRE_BUS 20  // D20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress RTC_Thermometer = { 0x28, 0x46, 0xBD, 0x19, 0x3, 0x0, 0x0, 0x35 };
DeviceAddress EXT_Thermometer = { 0x28, 0x33, 0x50, 0x7A, 0x5, 0x0, 0x0, 0xB9 };

// Device 0 Address: 2846BD1903000035
// Device 1 Address: 2833507A050000B9

#define TEMPERATURE_PRECISION 9

// #define ds oneWire

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

boolean GPS_OUT = false; // Включить и выключить NMEA Output

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
unsigned long PreviousInterval = 0;        // Для всех внутренних функций
unsigned long loopPreviousInterval = 0;    // Для управления GPS SetDateTime
unsigned long voltPreviousInterval = 0;    // Для вольтметра
unsigned long barPreviousInterval = 0;     // Для барометра для показа
unsigned long BarSavePreviousInterval = 0; // Для барометра сохранять
unsigned long updatePreviousInterval = 0;  // Для обновление данных с BMP085
unsigned long updTimeCur = 0;              // Для обновление текущий данных на графиках

unsigned long gpsTrackPI = 0;            // Каждые пять минут сохраняем GPS Position
unsigned long gps_out_pi = 0;            // Если GPS_OUT мигаем светодиодом на MCU

boolean GPS_OUT_LED = false;

int Contrast = 44;

boolean start = true; // Если была перегрузка.

char nmea; // NMEA Данные с порта

boolean page = false;  // Какая страница для очистки при переходе
boolean page2 = false; // Мы на второй странице

float tempC; // Температура внешнего датчика

// --------------------------- ESP8266 ------------------------------------

// Open the file from \arduino\hardware\arduino\avr\cores\arduino\HardwareSerial.h. 
// See the follow line in the HardwareSerial.h file.
// #define SERIAL_BUFFER_SIZE 64
// The default size of the buffer is 64. Change it into a bigger number, like 256 or more.

ESP8266 wifi(Serial);

char pip[16],aip[16];  // IP Addresess

// #define SSID1                "OS"
// #define PASSWORD1            "pass"

#define SSID2                "Romik"
#define PASSWORD2            "pass"

#define SSID                 "RomikTP"
#define PASSWORD             "pass"

boolean wifi_status = false;

char str_nmea[100];
unsigned int nmea_pos;
boolean nmea_start=false;
boolean nmea_ready = false;
boolean wifi_connected = false;
uint8_t mux_id;
boolean GPS_NMEA_OUT = false; // Разрешение на вывод NMEA over TCP/IP
  
// --------------------------------- SETUP ---------------------------------

void setup() {
  
  Wire.begin();  // Attach I2C Protocol
 
  delay(500);
  
  sensors.begin();
  
  sensors.setResolution(RTC_Thermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(EXT_Thermometer, TEMPERATURE_PRECISION);
  
  rtc.begin();
  
  // rtc.adjust(DateTime(__DATE__, __TIME__)); // Востоновить время.
  
  EEPROM_readAnything(0, configuration); // Чтения конфигурации
  
  Display = configuration.Display;             // Default DISPLAY_1
  Contrast = configuration.Contrast;           // Default 44
  GPS_OUT = configuration.GPS_OUT;             // Выводить NMEA в Bluetooth
  
  rtc.writeSqwPinMode(SquareWave1HZ); // Включаем синий светодиод на DS1307
  
  // delay(1000); // For BMP085 - Зачем не понятно  
  // setDateTime(); // Установка начального времени
 
  // dps.init(); == dps.init(MODE_STANDARD, 0, true); 
  // dps.init(MODE_STANDARD, 101850, false);
  
  pinMode(BT_CONNECT,INPUT); // Есть ли Bluetooth соединение
  
  dps.init(MODE_ULTRA_HIGHRES, 25000, true);  // Разрешение BMP180
  
  irrecv.enableIRIn();
   
  pinMode(CPU_LED,OUTPUT);
  
  if (!GPS_OUT) digitalWrite(CPU_LED,HIGH);

  lcd.init(EPSON);   // Initializes lcd, using an PHILIPSdriver
  lcd.contrast(Contrast);  // -51's usually a good contrast value
  lcd.clear(BLACK);  // clear the screen
  
  // rtc.writeSqwPinMode(OFF); // Для экономии 10 мА
  // digitalWrite(CPU_LED,HIGH);

  Serial.begin(57600); // ESP8266 Wi-Fi
  Serial1.begin(4800);  // GPS EM-406
 
  bt.begin(9600);       // Bluetooth
  
  // erase_eeprom_bmp085();  // Стереть все данные EEPROM BMP085  
  
  // Read_Data_BMP_EEPROM(); // Чтение всех данный из EEPROM

  dps.getTemperature(&Temperature);  // Температура
  dps.getPressure(&Pressure);        // Давление
  dps.getAltitude(&Altitude);        // Высота 
  
  sensors.requestTemperatures(); 
  tempC = sensors.getTempC(EXT_Thermometer);
  
  if (DEBUG) { // Все работает
    
    while(1) {   
     sensors.requestTemperatures(); 
     tempC = sensors.getTempC(EXT_Thermometer);
     bt.println(tempC);
     tempC = sensors.getTempC(RTC_Thermometer);
     bt.println(tempC);   
     delay(1000);
     bt.println("------------------------------");  
    }
    
  }
        
  bmp085_data.Press = Pressure/133.3;
  bmp085_data.Alt   = Altitude/100.0;
  bmp085_data.Temp  = tempC;
  
  start = true; // Для того чтобы вернуть сохраненый дисплай и обновить его.
  
  if (DEBUG) bt.println("Debug ON...");
  
  if (wifi.kick()) {  // Если Wi-Fi ESP8266 модуль подключен.
  
    wifi.setOprToStationSoftAP();
  
    strcpy(pip,"0.0.0.0");
    strcpy(aip,"0.0.0.0");
  
    lcd.clear(BLACK);

    if (!wifi_status) {    
     lcd.setStr("Try RomikTP",0,15,WHITE, BLACK);           
     wifi.joinAP(SSID, PASSWORD);
     delay(200);
     wifi.joinAP(SSID, PASSWORD);     
     lcd.clear(BLACK);
     get_ip();
     if (strcmp(aip,"0.0.0.0")!=0) wifi_status = true;    
     delay(300);  
    }
    
/*    if (!wifi_status) {
      lcd.clear(BLACK);
      lcd.setStr("Try OS",0,15,WHITE, BLACK);        
      wifi.joinAP(SSID1, PASSWORD1);
      delay(200);
      wifi.joinAP(SSID1, PASSWORD1);
      lcd.clear(BLACK);
      get_ip();
      if (strcmp(aip,"0.0.0.0") != 0) wifi_status = true;
      delay(300);
    }
*/

    if (!wifi_status) {
      lcd.clear(BLACK);
      lcd.setStr("Try Romik",0,15,WHITE, BLACK);        
      wifi.joinAP(SSID2, PASSWORD2);
      delay(200);
      wifi.joinAP(SSID2, PASSWORD2);
      lcd.clear(BLACK);
      get_ip();
      if (strcmp(aip,"0.0.0.0") != 0) wifi_status = true;
      delay(300);
    }
  
     delay(1000);
     lcd.clear(BLACK);
     
     wifi.enableMUX();
     wifi.startTCPServer(80);
     wifi.setTCPServerTimeout(10);
         
  } // End Of Wi-Fi Setup.
   
}

///////////////////////////////////////////////////////////////////////
//  MAIN LOOP 
///////////////////////////////////////////////////////////////////////

void loop() {
  
     if (Serial1.available()) {
    
       nmea = Serial1.read();
       gps.encode(nmea);
    
    if (GPS_OUT && (digitalRead(BT_CONNECT) == HIGH)) bt.print(nmea);      
    
    if (Serial.available()) {
      if (GPS_NMEA_OUT)  nmea_to_wifi(nmea);
     }
    
  if (nmea_ready == false) {
  
    if (nmea == '$')  { nmea_pos = 0; nmea_start=true; }    
    
    if (nmea_start == true) {
     str_nmea[nmea_pos] = nmea; 
     nmea_pos++; 
     if (nmea == '\n') { // End of NMEA string
       if (nmea_pos < 99) str_nmea[nmea_pos] = '\0'; else str_nmea[99] = '\0';
        nmea_start = false;
        if (strstr(str_nmea,"RMC")) {
          nmea_ready = true; 
        } else {
          str_nmea[nmea_pos] = '\0';
          nmea_pos = 0;
          nmea_start = false;
        }
     }
    }
    
    if (nmea_pos==99) { 
      str_nmea[nmea_pos] = '\0';
      nmea_pos =0;
      nmea_start =  false;
    }
  } // if nmea_ready == false
  
   } // If GPS Data is coming
     
     
   currentMillis = millis();

  if(currentMillis - updatePreviousInterval > 10000) {  // Каждые 10 секунд
   updatePreviousInterval = currentMillis;  

   dps.getTemperature(&Temperature);  // Температура
   dps.getPressure(&Pressure);        // Давление
   dps.getAltitude(&Altitude);        // Высота 

   sensors.requestTemperatures(); 
   tempC = sensors.getTempC(EXT_Thermometer);

   bmp085_data.Press = ( bmp085_data.Press + Pressure/133.3 ) / 2.0;
   bmp085_data.Alt   = ( bmp085_data.Alt + Altitude/100.0 ) / 2.0;
   bmp085_data.Temp  = ( bmp085_data.Temp + tempC ) / 2.0;

  }

   if (Display == DISPLAY_1) Analog_Time_Clock();
   if (Display == DISPLAY_2) SetupMenu();
   if (Display == DISPLAY_3) ShowData(start);
   if (Display == DISPLAY_4) ShowDataGPS(start);
   if (Display == DISPLAY_5) ShowDataSun(start);
   if (Display == DISPLAY_6) ShowDataVolt(start);
   if (Display == DISPLAY_8) ShowBMP085(start);
   
   if (Display == DISPLAY_10) ShowBMP085Temp(start); 
   if (Display == DISPLAY_11) ShowBMP085Alt(start); 
      
   start = false; // Если была перегрузка, чтобы не перересовывать всю графику
  
   if (irrecv.decode(&results)) {
    
     if (DEBUG) { bt.print("IR Code:"); bt.println(results.value); }
     
     if (results.value == DIS_UP && Display == DISPLAY_MENU) { // Вниз
       X_Menu--; if (X_Menu == 0) { page=true; X_Menu = MAX_MENU;  }
       if (X_Menu <= MAX_DISPLAY) page2=false;
       SetupMenu(); 
     }
     
     if (results.value == DIS_DOWN && Display == DISPLAY_MENU) { // Вверх
       X_Menu++; if (X_Menu > MAX_MENU) X_Menu=1;
       if (X_Menu > MAX_DISPLAY) { if (!page2) { page = true; page2=true; } }
       SetupMenu(); 
     }
     
     if (results.value == DIS_ENTER && Display == DISPLAY_MENU) { // Нажали ENTER
      switch (X_Menu) {      
       case 1: results.value = DISPLAY_1; break;
       case 2: results.value = DISPLAY_6; break;
       case 3: results.value = DISPLAY_3; break;
       case 4: results.value = DISPLAY_4; break;
       case 5: results.value = DISPLAY_5; break;
       case 6: results.value = DISPLAY_7; break;
       case 7: results.value = DISPLAY_8; break;       
       case 8: 
               if (!GPS_NMEA_OUT) {
                 if (GPS_OUT) GPS_OUT = false; else GPS_OUT = true; 
                 results.value = DISPLAY_2; 
                 configuration.GPS_OUT = GPS_OUT;
                 EEPROM_writeAnything(0, configuration);
               }
               break;
       case 9:  results.value = DISPLAY_10; break;
       case 10: results.value = DISPLAY_11; break;
       case 11: Read_Data_BMP_EEPROM(); break;
       case 12: get_ip(); break;
       case 13: 
                if (!GPS_OUT) {
                  if (GPS_NMEA_OUT) GPS_NMEA_OUT = false; else GPS_NMEA_OUT = true;
                  results.value = DISPLAY_2;
                }
                break;

      }
     }  
          
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
     case DI_1: results.value = DISPLAY_1; break;
     case DI_2: results.value = DISPLAY_2; break;
     case DI_3: results.value = DISPLAY_3; break;
     case DI_4: results.value = DISPLAY_4; break;
     case DI_5: results.value = DISPLAY_5; break;
     case DI_6: results.value = DISPLAY_6; break;
     case DI_7: results.value = DISPLAY_7; break;
     case DI_8: results.value = DISPLAY_8; break;
     case DI_9: results.value = DISPLAY_9; break;
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

     case DISPLAY_7:
      GPS_Track_Output();
      break;
      
     case DISPLAY_8:
      Display = DISPLAY_8;
      configuration.Display = DISPLAY_8;
      EEPROM_writeAnything(0, configuration);
      lcd.clear(BLACK);
      ShowBMP085(true);
      break;

     case DISPLAY_10:
      Display = DISPLAY_10;
      configuration.Display = DISPLAY_10;
      EEPROM_writeAnything(0, configuration);
      lcd.clear(BLACK);
      ShowBMP085Temp(true);
      break;

     case DISPLAY_11:
      Display = DISPLAY_11;
      configuration.Display = DISPLAY_11;
      EEPROM_writeAnything(0, configuration);
      lcd.clear(BLACK);
      ShowBMP085Alt(true);
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
      if (!GPS_NMEA_OUT) {
        if (GPS_OUT) GPS_OUT = false; else GPS_OUT = true;
        configuration.GPS_OUT = GPS_OUT;
        EEPROM_writeAnything(0, configuration);
        lcd.clear(BLACK);
        SetupMenu();                
      }
      break;
      
    }
    
    irrecv.resume(); 
   }     


  currentMillis = millis();

 // --------------------------- GPS -----------------------
  
  if(currentMillis - gpsTrackPI > (FIVE_MINUT/2)) { // Каждые 2.5 минут Save GPS Position
   gpsTrackPI = currentMillis;  
   Save_GPS_Pos();  // Save GPS Position
   if (!GPS_OUT && !GPS_NMEA_OUT) { 
    if (wifi_status) {
     if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
       if (nmea_ready) { send_nmea_wifi(str_nmea); nmea_ready = false; }
     }
    }
   }
  }
  
  if(currentMillis - BarSavePreviousInterval > (FIVE_MINUT*3)) { //*3 //*4 Каждые 20 минут Save BAR Parameters 
   BarSavePreviousInterval = currentMillis;
   Save_Bar_Data(); // Save BMP_085 Data  
  }
 
  if(currentMillis - loopPreviousInterval > FIVE_MINUT) {  // Каждые 5 минут [300000]
   loopPreviousInterval = currentMillis;     
   
   if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
     rtc.writeSqwPinMode(OFF);             // Выключаем синий светодиод на DS1307
     set_GPS_DateTime();     
   }
   
     if (battary() < 4.0) { // Если идет зарядка то можно включить
      rtc.writeSqwPinMode(OFF); 
     } else {
      rtc.writeSqwPinMode(SquareWave1HZ);   // Включаем синий светодиод на DS1307
     }
  }
  
 if (GPS_OUT || GPS_NMEA_OUT) {  
  if(currentMillis - gps_out_pi > 250) {    
   gps_out_pi = currentMillis;  
   if (GPS_OUT_LED) {  
    digitalWrite(CPU_LED,HIGH); 
    GPS_OUT_LED = false; 
   } else { 
    digitalWrite(CPU_LED,LOW); 
    GPS_OUT_LED = true; 
   }
  }
 }
 
}

// --------------------------- GPS SAVE POS FOR GPS TRACKER --------------------------------------

void Save_GPS_Pos( void ) {

  unsigned long GPS_EEPROM_POS;

 if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
   if (gps.location.isUpdated()) {
  
  EEPROM_readAnything(0, configuration); // Чтения конфигурации
  
  GPS_EEPROM_POS = configuration.Last_GPS_Pos;  

   gps_tracker.lats = gps.location.lat();
   gps_tracker.lngs = gps.location.lng();
  
   gps_tracker.days    = gps.date.day();
   gps_tracker.months  = gps.date.month();
   gps_tracker.years   = gps.date.year();
   gps_tracker.minutes = gps.time.minute();
   gps_tracker.hours   = gps.time.hour();
          
     const byte* p = (const byte*)(const void*)&gps_tracker;
     for (unsigned int i = 0; i < sizeof(gps_tracker); i++) 
     eeprom256.writeByte(GPS_EEPROM_POS++, *p++);
        
     if (GPS_EEPROM_POS > (EE24LC256MAXBYTES - (sizeof(gps_tracker)+1) ) ) {
      configuration.Last_GPS_Pos = 0;
      GPS_EEPROM_POS = 0;
     } else {
      configuration.Last_GPS_Pos = GPS_EEPROM_POS; // Следующая ячейка памяти в EEPROM
     }
     
     EEPROM_writeAnything(0, configuration);

   } // if GPS is Updated
  } // if GPS is OK
}
// --------------------------- GPS Track Output --------------------------------------------------

void GPS_Track_Output( void ) {
  
  unsigned long address = 0;
  int count = 0;
  char f[20];
  
  if (digitalRead(BT_CONNECT) == HIGH && !GPS_OUT) {      
  
    lcd.clear(BLACK);
    
    DateTime now = rtc.now();
 
  bt.println(F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"));
  bt.println(F("<gpx xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" version=\"1.0\""));
  bt.println(F("xmlns=\"http://www.topografix.com/GPX/1/0\" creator=\"Polar WebSync 2.3 - www.polar.fi\""));
  bt.println(F("xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd\">"));
  bt.print(F("<time>"));
  bt.print(now.year());
  bt.print("-");
  if (now.month() < 10) bt.print(F("0")); bt.print(now.month());
  bt.print("-");
  if (now.day() < 10) bt.print(F("0")); bt.print(now.day());
  bt.print("T");
  if (now.hour() < 10) bt.print(F("0")); bt.print(now.hour());
  bt.print(":");
  if (now.minute() < 10) bt.print(F("0")); bt.print(now.minute());
  bt.print(":");
  if (now.second() < 10) bt.print(F("0")); bt.print(now.second());
  bt.println(F("Z</time>"));
  bt.println(F("<trk>"));
  bt.println(F("<name>GPS Track by Roma Kuzmin</name>"));
  bt.println(F("<trkseg>"));
 
  lcd.setRect(70,20, 100, 100, 0, WHITE); // box   
  int ix = 0;
  
  while (address < (EE24LC256MAXBYTES - (sizeof(gps_tracker)+1) ) )  {
  
   //sprintf(f,"Output: %.4d",address);
   //lcd.setStr(f,3,3,YELLOW,BLACK);
   
   lcd.setLine(70,20+ix,100,20+ix,WHITE); // Заливаем i=0 to i=79
   ix++;
   if (ix==80) { 
    lcd.setRect(70,20,101,101,1,BLACK); // Erase All
    lcd.setRect(70,20,100,100,0,WHITE); // box   
    ix=0;
   }
    
   byte* pp = (byte*)(void*)&gps_tracker; 
   for (unsigned int i = 0; i < sizeof(gps_tracker); i++)
    *pp++ = eeprom256.readByte(address++);
  
     if (gps_tracker.days == 0 && gps_tracker.months == 0 && gps_tracker.years == 0) break;
     if (gps_tracker.days == 255 && gps_tracker.months == 255) break;
     
//     bt.print(count++);             bt.print(';');
//     bt.print(gps_tracker.days);    bt.print(';');
//     bt.print(gps_tracker.months);  bt.print(';');
//     bt.print(gps_tracker.years);   bt.print(';');     
//     bt.print(gps_tracker.hours);   bt.print(';');     
//     bt.print(gps_tracker.minutes); bt.print(';');
//     bt.print(gps_tracker.lats,6);  bt.print(';');
//     bt.print(gps_tracker.lngs,6);  
//     bt.println();
     
      bt.print("<trkpt lat=\""); 
      bt.print(gps_tracker.lats,6); 
      bt.print("\" lon=\""); 
      bt.print(gps_tracker.lngs,6); 
      bt.println("\">");

      bt.print("<time>"); 
      bt.print(gps_tracker.years); 
      bt.print("-");
      if (gps_tracker.months < 10) bt.print(F("0"));
      bt.print(gps_tracker.months);
      bt.print("-");
      if (gps_tracker.days < 10) bt.print(F("0"));
      bt.print(gps_tracker.days); 
      bt.print("T");
      if (gps_tracker.hours < 10) bt.print(F("0"));
      bt.print(gps_tracker.hours);
      bt.print(":");
      if (gps_tracker.minutes < 10) bt.print(F("0"));
      bt.print(gps_tracker.minutes);
      bt.print(":00Z");
      bt.println("</time>");
      bt.println("</trkpt>");
    }

   bt.println(F("</trkseg>"));
   bt.println(F("</trk>"));
   bt.println(F("</gpx>"));
    
   }  
   
   lcd.clear(BLACK);
   Display = DISPLAY_2;
  
}

// --------------------------- Output BMP_085 DATA from EEPROM -----------------------------------

void Read_Data_BMP_EEPROM( void ) {  // MENU:BarEepromOut
  
   Average<double> bar_data(96); // Вычисление максимального и минимального значения
   Average<double> alt_data(96); // Вычисление максимального и минимального значения
   Average<double> tem_data(96); // Вычисление максимального и минимального значения
   
   BAR_EEPROM_POS = 0;

   lcd.setRect(70+10,20, 100+10, 116, 0, WHITE); // Рисуем квадрат пустой
 
   bt.println("--------------- START -----------------------");
    
   DateTime now = rtc.now();
    
   for(byte j=0;j<96;j++) {
     
   lcd.setLine(70+10,20+j,100+10,20+j,WHITE); // Заливаем i=0 to i=95
                 
    byte* pp = (byte*)(void*)&bmp085_data_out; 
    for (unsigned int i = 0; i < sizeof(bmp085_data_out); i++)
     *pp++ = eeprom32.readByte(BAR_EEPROM_POS++);
    
    if (bmp085_data_out.Press != 0.0) bar_data.push(bmp085_data_out.Press);
    
     alt_data.push(bmp085_data_out.Alt);
     tem_data.push(bmp085_data_out.Temp);         
    
    bt.print(bmp085_data_out.Press);      bt.print(";");
    bt.print(bmp085_data_out.Alt);        bt.print(";");
    bt.print(bmp085_data_out.Temp);       bt.print(";");
    bt.print(bmp085_data_out.unix_time);  bt.print(";");
    bt.print(now.unixtime() - bmp085_data_out.unix_time); bt.print(";");
    bt.print(now.unixtime()); bt.print(";");    

    DateTime eeTime (bmp085_data_out.unix_time);

    bt.print(eeTime.year()); bt.print("-");
    bt.print(eeTime.month());bt.print("-");
    bt.print(eeTime.day());

    bt.print("  ");

    bt.print(eeTime.hour());bt.print(":");
    bt.print(eeTime.minute());bt.print(":");
    bt.print(eeTime.second());
    
    
    bt.print(" MAP: ");
    bt.print(map(bmp085_data_out.Press,bar_data.minimum(),bar_data.maximum(),106,1));
    bt.print(" ");
    bt.print(map(bmp085_data_out.Alt  ,alt_data.minimum(),alt_data.maximum(),106,1));
    bt.print(" ");
    bt.print(map(bmp085_data_out.Temp ,tem_data.minimum(),tem_data.maximum(),106,1));
    
    
    bt.println();
          
   }
   
   bt.println("-----------MAX and MIN-------------");
   bt.println(bar_data.maximum());
   bt.println(bar_data.minimum());
   bt.println("");
   bt.println(alt_data.maximum());
   bt.println(alt_data.minimum());
   bt.println("");
   bt.println(tem_data.maximum());
   bt.println(tem_data.minimum());
   
   bt.println("--------------- STOP -----------------------");
   
   BAR_EEPROM_POS = 0;
    
   lcd.setRect(70+10,20, 101+10, 117, 1, BLACK); // Удаляем весь квадрат
    
}
// --------------------------- Save Barometer Data to EEPROM -------------------------------------

void Save_Bar_Data( void ) {

    // 48 часов * 60 минут = 2880 Минут
    // 2880 минут / 30 минут = 96 Ячеек
    // (UnixTime / 1800) % 96 = номер ячейки

   dps.getTemperature(&Temperature);  // Температура
   dps.getPressure(&Pressure);        // Давление
   dps.getAltitude(&Altitude);        // Высота 
   
   DateTime now = rtc.now();
  
   bmp085_data.unix_time = now.unixtime(); 
   
   // DEBUG
   
   if (DEBUG) {
     bt.print("h:"); bt.println(now.hour());
     bt.print("m:"); bt.println(now.minute());
     bt.print("p:"); bt.println((bmp085_data.unix_time/1800)%96);
   }
   
   BAR_EEPROM_POS = ( (bmp085_data.unix_time/1800)%96 ) * sizeof(bmp085_data); // Номер ячейки памяти.
  
   sensors.requestTemperatures(); 
   tempC = sensors.getTempC(EXT_Thermometer);  
       
   bmp085_data.Press = ( bmp085_data.Press + Pressure/133.3 ) / 2.0;
   bmp085_data.Alt   = ( bmp085_data.Alt + Altitude/100.0 ) / 2.0;
   bmp085_data.Temp  = ( bmp085_data.Temp + tempC ) / 2.0;
     
   if (DEBUG) {
    bt.println(BAR_EEPROM_POS/sizeof(bmp085_data));
    bt.println(bmp085_data.unix_time);
    bt.println(now.hour());
    bt.println(now.minute());   
    bt.println("-------");
   }
   
   const byte* p = (const byte*)(const void*)&bmp085_data;
   for (unsigned int i = 0; i < sizeof(bmp085_data); i++) 
    eeprom32.writeByte(BAR_EEPROM_POS++,*p++);
   
}

// --------------------------- Erase DATA EEPROM 32 for BMP085 -----------------------------------

void erase_eeprom_bmp085( void ) {
  
  char f[10];
  
  BAR_EEPROM_POS = 0;
   
  bmp085_data.Press     = 0.0;
  bmp085_data.Alt       = 0.0;
  bmp085_data.Temp      = 0.0;
  bmp085_data.unix_time = 0;

  lcd.clear(BLACK);
  
  while(BAR_EEPROM_POS < (EE24LC32MAXBYTES - (sizeof(bmp085_data) + 1))) {
   
   const byte* p = (const byte*)(const void*)&bmp085_data;
   for (unsigned int i = 0; i < sizeof(bmp085_data); i++)
    eeprom32.writeByte(BAR_EEPROM_POS++,*p++);
 
    sprintf(f,"%.3d",BAR_EEPROM_POS);
    lcd.setStr(f,2,2,YELLOW,BLACK);
  }
    
  BAR_EEPROM_POS = 0;
  
  lcd.clear(BLACK);
   
}

// -------------------------- Измерение входного напряжения от батарейки 3.7V Батарейка 4.x - Зарядка ----

float battary( void ) {
  
  int sensorValue = analogRead(A4); // PA4/D28
  float voltage = sensorValue * (5.0 / 1023.0);
  return(voltage);
  
}

void draw_battary( boolean type ) {
  
  // Type == true  - Normal
  // Type == false - Clear box
  
  if (type) {
   lcd.setRect(115,40, 125, 55, 0, WHITE); // box   
   lcd.setRect(117,55, 123, 58, 1, WHITE); // box +   
   lcd.setRect(115,40, 125, 45, 1, WHITE); // box full  
  } else {
   lcd.setRect(115,40, 125, 55, 0, BLACK); // box   
   lcd.setRect(117,55, 123, 58, 1, BLACK); // box +   
   lcd.setRect(115,40, 125, 45, 1, BLACK); // box full      
  }
   
}

// --------------------------------------------- Функция для Аналоговых часов --------------------

void displayDigitalTime(byte h, byte m, byte s, byte d,byte mo,int ye)
{
    char timeChar[12];
  
    sprintf(timeChar, "%.2d:%.2d:%.2d", h, m, s);
    lcd.setStr(timeChar,105,5, C_COLOR, BACKGROUND);
    sprintf(timeChar, "%.2d/%.2d", d, mo);
    lcd.setStr(timeChar,105,79, C_COLOR, BACKGROUND);
}

void drawClock()
{

  lcd.setCircle(CLOCK_CENTER, 66, CLOCK_RADIUS,   C_COLOR);
  lcd.setCircle(CLOCK_CENTER, 66, CLOCK_RADIUS+1, C_COLOR);
  lcd.setCircle(CLOCK_CENTER, 66, CLOCK_RADIUS+2, C_COLOR);
  
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

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

}

// ---------------------------- Установка времени через GPS ------------------------

void set_GPS_DateTime() {
   
 if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {

   DateTime utc = (DateTime (gps.date.year(), 
                             gps.date.month(), 
                             gps.date.day(),
                             gps.time.hour(),
                             gps.time.minute(),
                             gps.time.second()) + 60 * 60 * UTC);
                                
  rtc.adjust(DateTime(utc.unixtime()));
  
 }
  
}



// ---------------------------------- Отображаем аналоговые часы -----------------------------

void Analog_Time_Clock( void ) {
    
  if(currentMillis - PreviousInterval > 1000) { 
    PreviousInterval = currentMillis;  
  
    DateTime now = rtc.now();
    
    byte seconds = now.second();
    byte minutes = now.minute();
    byte hours   = now.hour();
   
    drawClock();
   
    displayAnalogTime(hours, minutes, seconds);
    displayDigitalTime(hours, minutes, seconds, now.day(),now.month(),now.year());
    
  }
  
}

// -------------------------------- Show Data on Display 3 -----------------------------------

void ShowData(boolean s) {

   char output[20];
   char f[30];
   
  if ((currentMillis - PreviousInterval > 1000) || (s == true) ) { 
   PreviousInterval = currentMillis;  

   dps.getTemperature(&Temperature);
   dps.getPressure(&Pressure); 
   dps.getAltitude(&Altitude); 
   
  sensors.requestTemperatures(); 
  tempC = sensors.getTempC(RTC_Thermometer);  
  
   dtostrf(Altitude/100.0, 4, 2, output);
   strcpy(f,"Alt: ");
   strcat(f,output);
   lcd.setStr(f,0,2,WHITE, BLACK);
    
   dtostrf(Pressure/133.3, 4, 2, output);
   strcpy(f,"Press: ");
   strcat(f,output);
   lcd.setStr(f,15,2,WHITE, BLACK);

   dtostrf(Temperature*0.1, 4, 2, output);
   strcpy(f,"T[bar]: ");
   strcat(f,output);
   lcd.setStr(f,30,2,WHITE, BLACK);
       
   dtostrf(battary(), 4, 2, output);
   strcpy(f,"Vin: ");
   strcat(f,output);
   lcd.setStr(f,45,2,WHITE, BLACK);       
   
   DateTime now = rtc.now();
    
   byte seconds = now.second();
   byte minutes = now.minute();
   byte hours = now.hour();
    
   byte monthDay = now.day();
   byte month = now.month();
    
   sprintf(output, "%.2d:%.2d:%.2d", hours, minutes, seconds);
   strcpy(f,"Time: ");
   strcat(f,output);
   lcd.setStr(f,60,2,WHITE, BLACK);       

   sprintf(output,"%.2d/%.2d/%.4d", monthDay, month, now.year());
   strcpy(f,"Date: ");
   strcat(f,output);
   lcd.setStr(f,75,2,WHITE, BLACK);       
     
   // tempC = sensors.getTempC(EXT_Thermometer);
   
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

// ------------------------ Setup Menu ----------------------------------

void SetupMenu( void ) {
  
  char f[MAX_MENU][20];
  byte pos = 0;
  byte y = 0;
  
  unsigned int text[MAX_MENU];
  unsigned int   bg[MAX_MENU];
  
  for(byte i=0;i<MAX_MENU;i++) {
    text[i] = WHITE;
    bg[i] = BLACK;
  }
  
  text[X_Menu-1] = BLACK; 
    bg[X_Menu-1] = WHITE;
  
  // Длина 15 символов
  
   strcpy(f[0],"1.Analog Clock ");      
   strcpy(f[1],"2.Voltmeter    ");   
   strcpy(f[2],"3.BMP/GPS Data ");   
   strcpy(f[3],"4.GPS Data     ");   
   strcpy(f[4],"5.Sun Set/Rise ");   
   strcpy(f[5],"6.GPX Track Out");   
   strcpy(f[6],"7.Barometer    ");   
   strcpy(f[7],"8.GPS NMEA     ");
   strcpy(f[8],"9.Temperature  ");
   strcpy(f[9],"A.Altimeter    ");
  strcpy(f[10],"B.BarEepromOut ");
  strcpy(f[11],"C.Wi-Fi Stat   ");
  strcpy(f[12],"D.Wi-Fi NMEA   ");

   if (DEBUG) { bt.print(F("Menu Position:")); bt.println(X_Menu); }

   if ( page ) { page = false ; lcd.clear(BLACK); }
      
   if (X_Menu > MAX_DISPLAY) { pos = round(X_Menu / MAX_DISPLAY) * MAX_DISPLAY; } 
   
   if (DEBUG) { bt.print(F("Menu Position:")); bt.println(pos); }
   
   for(byte j=0;j<MAX_DISPLAY;j++) {
     
     if (pos > MAX_MENU-1) break;
     
     if ((GPS_OUT && pos == 7) || (GPS_NMEA_OUT && pos == 12)) // Только для просмотра что влючено NMEA OUTPUT
      lcd.setStr(f[pos],y*15,10,WHITE,RED);
     else 
      lcd.setStr(f[pos], y*15, 10,text[pos],bg[pos]);
      
      pos++;
      y++;
     
   }

   Display = DISPLAY_MENU; 

   
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
      
      sprintf(f,"gTime:%.2d:%.2d:%.2d",gps.time.hour(),gps.time.minute(),gps.time.second());
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
  
  DateTime now = rtc.now(); // Time from DS1307
  
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

   t = mySunrise.Rise(now.month(),now.day()); // Month, Day !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   
   if(t >= 0) {
    h_rise = mySunrise.Hour();
    m_rise = mySunrise.Minute();
   } else { h_rise = 0; m_rise = 0; }    
   
   t = mySunrise.Set(now.month(),now.day()); // Month, Day !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   
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
            
      sprintf(f,"gTime:%.2d:%.2d:%.2d",gps.time.hour(),gps.time.minute(),gps.time.second());
      lcd.setStr(f,60+7,2,WHITE, BLACK);              
      sprintf(f,"gDate:%.2d/%.2d/%.2d",gps.date.day(),gps.date.month(),gps.date.year());
      lcd.setStr(f,75+7,2,WHITE, BLACK);        
      
      DateTime utc = (DateTime (gps.date.year(), 
                                gps.date.month(), 
                                gps.date.day(),
                                gps.time.hour(),
                                gps.time.minute(),
                                gps.time.second()) + 60 * 60 * UTC);

      // Convert GPS to LocalTime + UTC
      
      sprintf(f,"UTC:%.2d:%.2d:%.2d",utc.hour(),utc.minute(),utc.second());
      lcd.setStr(f,90+10,2,WHITE, RED);
              

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

  char f[10];
  
   if ((currentMillis - PreviousInterval > 1000) || (s == true) ) { 
     PreviousInterval = currentMillis;  
     dtostrf(battary(),4,2, f);
     lcd.setStr(f,0,90,RED,BLACK);  
   }
  
  if ((currentMillis - voltPreviousInterval > FIVE_MINUT/2) || (s == true) ) { 
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

  int x = map(battary(),0.0,5.0,106,0);
  lcd.setLine(x,y_volts,106,y_volts, WHITE);
   
  y_volts++; if (y_volts > 130) { lcd.clear(BLACK); y_volts=15; start=true; }
  
  }  
  
}

// ---------------- Barometer Graphics ------------------------

void ShowBMP085(boolean s) {

 if ((currentMillis - barPreviousInterval > FIVE_MINUT/2) || (s == true) ) {  
      barPreviousInterval = currentMillis;      

   DateTime now = rtc.now(); 
   
   byte current_position = (now.unixtime()/1800)%96;  
  
   Average<double> bar_data(96); // Вычисление максимального и минимального значения
   
   double barArray[96];   
   
   BAR_EEPROM_POS = 0;
    
   for(byte j = 0;j < 96; j++) {           
  
     byte* pp = (byte*)(void*)&bmp085_data_out; 
  
    for (unsigned int i = 0; i < sizeof(bmp085_data_out); i++)
     *pp++ = eeprom32.readByte(BAR_EEPROM_POS++); 
     
    if ((now.unixtime() - bmp085_data_out.unix_time) < TWO_DAYS) {
   
       barArray[j] = bmp085_data_out.Press; 
       bar_data.push(bmp085_data_out.Press);
     
    } else {
      
       barArray[j] = 0.0;
 
    }
    
     if (DEBUG) {
        bt.print(now.unixtime() - bmp085_data_out.unix_time);  // Debug
        bt.print(" -- ");                                      // Debug
        bt.println(barArray[j]);                               // DEBUG
      }
   }

       if (DEBUG) bt.println("-------------------");  // DEBUG
        

   BAR_EEPROM_POS = 0;

   lcd.setLine(1,30,130,30,WHITE);   // Линия по верикали
   lcd.setLine(107,0,107,129,WHITE); // Линия по горизонтали

   // Давление     

   char f[10];
   int x;
   
   dps.getTemperature(&Temperature);
   dps.getPressure(&Pressure); 

   sprintf(f,"%d",(int)bar_data.maximum());
   lcd.setStr(f,0,3,WHITE,BLACK);
   
   sprintf(f,"%d",(int)bar_data.mean());    
   lcd.setStr(f,42,3,GREEN,BLACK);
   
   sprintf(f,"%d",(int)bar_data.minimum());    
   lcd.setStr(f,85,3,WHITE,BLACK);
   
   sprintf(f,"%d",(int)(Pressure/133.3));     
   lcd.setStr(f,107,3,YELLOW,BLACK);   // Текущие давление
  
   // Время
      
   if ( battary() < 4.0 ) {   // Рисуем батарейку
    strcpy(f,"      ");
    lcd.setStr(f,107,33,RED,BLACK);
    draw_battary(true);
   } else { 
    draw_battary(false);
    strcpy(f,"2 Days");
    lcd.setStr(f,107,33,RED,BLACK);
   }
   
   sprintf(f,"%.2d:%.2d",now.hour(),now.minute());
   lcd.setStr(f,107,88,GREEN,BLACK);
    
   int y_pres = 127;
    
   for(byte j=0;j<96;j++) {
     
    if (j != 0) {
     x = map(barArray[current_position],bar_data.minimum()-1,bar_data.maximum()+1,106,1);
    } else {
     x = map(Pressure/133.3,bar_data.minimum()-1,bar_data.maximum()+1,106,1); // Текущие значение
    }

    lcd.setLine(0,y_pres,106,y_pres, BLACK); // Стереть линию
     
    if (barArray[current_position] != 0.0) {     
     lcd.setLine(x,y_pres,106,y_pres, WHITE); // Нарисовать данные    
    }
    
    if (current_position == 0) current_position = 96;
    
    current_position--; 
    
    y_pres--;
    
   } 
   
    // Read_Data_BMP_EEPROM();  
  
  }  
  
}


// ---------------- Temperature Graphics ------------------------

void ShowBMP085Temp(boolean s) {

   char f[10];
   int x;
   
   // tempC = sensors.getTempC(EXT_Thermometer);

   DateTime now = rtc.now();    
   
  if ((currentMillis - updTimeCur > 10000) || (s == true) ) {  
   updTimeCur = currentMillis;      
   
   if (DEBUG) bt.println(tempC);
   
   sprintf(f,"%3d",(int)(tempC));     
   lcd.setStr(f,107,3,YELLOW,BLACK);   // Текущие значение температуры
 
    sprintf(f,"%.2d:%.2d",now.hour(),now.minute());
    lcd.setStr(f,107,88,GREEN,BLACK);
   }   
   
 if ((currentMillis - barPreviousInterval > FIVE_MINUT/2) || (s == true) ) {  
      barPreviousInterval = currentMillis;      
  
   Average<double> temp_data(96); // Вычисление максимального и минимального значения
   
   double tempArray[96];   
   
   BAR_EEPROM_POS = 0;
    
   for(byte j = 0;j < 96; j++) {           
    byte* pp = (byte*)(void*)&bmp085_data_out; 
    for (unsigned int i = 0; i < sizeof(bmp085_data_out); i++)
     *pp++ = eeprom32.readByte(BAR_EEPROM_POS++);    
    if ((now.unixtime() - bmp085_data_out.unix_time) < TWO_DAYS) {
     tempArray[j] = bmp085_data_out.Temp;   
     temp_data.push(bmp085_data_out.Temp);
    } else tempArray[j] = 0.0;
    if (DEBUG) bt.println(tempArray[j]);
   }

   BAR_EEPROM_POS = 0;

   lcd.setLine(1,30,130,30,WHITE);   // Линия по верикали
   lcd.setLine(107,0,107,129,WHITE); // Линия по горизонтали

   // Давление     
   
   // dps.getTemperature(&Temperature); // Текущие значение температуры
   
   // float tempC = getTemperature(EXT_Thermometer);
   //       tempC = f2c(tempC);
   
   sprintf(f,"%d",(int)temp_data.maximum());
   lcd.setStr(f,0,3,WHITE,BLACK);
   
   sprintf(f,"%d",(int)temp_data.mean());    
   lcd.setStr(f,42,3,GREEN,BLACK);
   
   sprintf(f,"%d",(int)temp_data.minimum());    
   lcd.setStr(f,85,3,WHITE,BLACK);
   
   sprintf(f,"%3d",(int)(tempC));     
   lcd.setStr(f,107,3,YELLOW,BLACK);   // Текущие значение температуры
  
   // Время
     
   if ( battary() < 4.0 ) {   // Рисуем батарейку
    strcpy(f,"      ");
    lcd.setStr(f,107,33,RED,BLACK);
    draw_battary(true);
   } else { 
    draw_battary(false);
    strcpy(f,"2 Days");
    lcd.setStr(f,107,33,RED,BLACK);
   }
   
 //  sprintf(f,"%.2d:%.2d",now.hour(),now.minute());
 //  lcd.setStr(f,107,88,GREEN,BLACK);
    
   int y_temp = 127;
    
   unsigned int current_position = (now.unixtime()/1800)%96;
   
   for(byte j=0;j<96;j++) {
    
    if (j != 0) x = map(tempArray[current_position],temp_data.minimum(),temp_data.maximum(),106,1);  
    else x = map(tempC,temp_data.minimum(),temp_data.maximum(),106,1);

    lcd.setLine(0,y_temp,106,y_temp, BLACK); // Стереть линию
     
    if (tempArray[current_position] != 0.0) {     
     lcd.setLine(x,y_temp,106,y_temp, WHITE); // Нарисовать данные    
    }
    
    if (current_position == 0) current_position = 96;
    
    current_position--; 
    
    y_temp--;

   } 
  
  }  
  
}


// ---------------- Altimetr Graphics ------------------------

void ShowBMP085Alt(boolean s) {

 if ((currentMillis - barPreviousInterval > FIVE_MINUT/2) || (s == true) ) {  
      barPreviousInterval = currentMillis;      

   DateTime now = rtc.now();    
  
   Average<double> alt_data(96); // Вычисление максимального и минимального значения
   
   double altArray[96];   
   
   BAR_EEPROM_POS = 0;
    
   for(byte j = 0;j < 96; j++) {           
    byte* pp = (byte*)(void*)&bmp085_data_out; 
    for (unsigned int i = 0; i < sizeof(bmp085_data_out); i++)
     *pp++ = eeprom32.readByte(BAR_EEPROM_POS++);    
    if ((now.unixtime() - bmp085_data_out.unix_time) < TWO_DAYS) {
     altArray[j] = bmp085_data_out.Alt;   
     alt_data.push(bmp085_data_out.Alt);
    } else altArray[j] = 0.0;
    if (DEBUG) bt.println(altArray[j]);
   }

   BAR_EEPROM_POS = 0;

   lcd.setLine(1,30,130,30,WHITE);   // Линия по верикали
   lcd.setLine(107,0,107,129,WHITE); // Линия по горизонтали

   // Давление     

   char f[10];
   int x;
   
    dps.getTemperature(&Temperature);
    dps.getPressure(&Pressure); 
    dps.getAltitude(&Altitude); // Текущие значение Высоты

   sprintf(f,"%d",(int)alt_data.maximum());
   lcd.setStr(f,0,3,WHITE,BLACK);
   
   sprintf(f,"%d",(int)alt_data.mean());    
   lcd.setStr(f,42,3,GREEN,BLACK);
   
   sprintf(f,"%d",(int)alt_data.minimum());    
   lcd.setStr(f,85,3,WHITE,BLACK);
   
   sprintf(f,"%d",(int)(Altitude/100.0));     
   lcd.setStr(f,107,3,YELLOW,BLACK);   // Текущие значение температуры
  
   // Время
      
   if ( battary() < 4.0 ) {   // Рисуем батарейку
    strcpy(f,"      ");
    lcd.setStr(f,107,33,RED,BLACK);
    draw_battary(true);
   } else { 
    draw_battary(false);
    strcpy(f,"2 Days");
    lcd.setStr(f,107,33,RED,BLACK);
   }
   
   sprintf(f,"%.2d:%.2d",now.hour(),now.minute());
   lcd.setStr(f,107,88,GREEN,BLACK);
    
   int y_alt = 127;
   
   byte current_position = (now.unixtime()/1800)%96;
   
   for(byte j=0;j<96;j++) {
    
    if (j != 0) x = map(altArray[current_position],alt_data.minimum(),alt_data.maximum(),106,1);  
    else x = map(Altitude/100.0,alt_data.minimum(),alt_data.maximum(),106,1);

    lcd.setLine(0,y_alt,106,y_alt, BLACK); // Стереть линию
     
    if (altArray[current_position] != 0.0) {     
     lcd.setLine(x,y_alt,106,y_alt, WHITE); // Нарисовать данные    
    }
    
    if (current_position == 0) current_position = 96;
    
    current_position--; 
    
    y_alt--;

   } 
  
  }  
  
}

// Новая функция

void erase_gps_track( void ) {
   
 //  lcd.clear(BLACK);
  
 lcd.setRect(70,20, 100, 100, 0, WHITE); // box   
 
 for(int i=0;i<80;i++) {
//  lcd.setRect(70,20, 100, 20+i, 1, WHITE); // 
  lcd.setLine(70,20+i,100,20+i,WHITE); // Заливаем i=0 to i=79
 }
 
 delay(1000);
 
 lcd.setRect(70,20, 101, 101, 1, BLACK); // Erase All
 
  
}

// --------------------------------------------------------------------------------------------------

void get_ip( void ) {
  
    char s[128];
    char *token;
    const char delimiters[] = "\":";
    unsigned long start;
    int  i = 0;
    
    lcd.clear(BLACK);
    lcd.setStr("IP Addresses",0,10,WHITE, BLACK);  

        start = millis();
        Serial.print("AT+CIFSR\r\n");
        while (millis() - start < 3000) {
          if (Serial.available()) { 
            s[i] = Serial.read(); 
             i++; 
             if (i == 127) { 
              s[i] = '\0'; 
              break; 
             }
          }
        }
        s[i] = '\0';
    
    token = strtok (s, delimiters);     

    while(token != NULL) {
      if  (strstr(token,"PIP")) strcpy(pip,strtok (NULL, delimiters)); 
      if  (strstr(token,"AIP")) strcpy(aip,strtok (NULL, delimiters)); 
      token = strtok (NULL, delimiters);   
    }    
        lcd.setStr(pip,15,10,WHITE, BLACK);  
        lcd.setStr(aip,30,10,WHITE, BLACK);  
                
        if (strcmp(aip,"0.0.0.0") != 0) {
          lcd.setStr("Wi-Fi Connect  ",45,10,WHITE, BLACK);          
        } else {
          lcd.setStr("Wi-Fi Error    ",45,10,WHITE, BLACK);  
        }
  }

// ---------------------------------- Send GPS over HTTP to a.pajero.su -------------------

void send_nmea_wifi(char *out_nmea ) {
   
  if (DEBUG) bt.print(out_nmea);
  
  int i;
 
  for (i=0;i<strlen(out_nmea);i++) {
    if (out_nmea[i] == '\r') out_nmea[i] = '\0';
  }
  
  String host = "62.113.122.4";
  
  uint32_t port = 80;
  
  static uint8_t muxid = 0; 
  
  wifi.createTCP(muxid,host,port);
  
  char snd[128];
   
    strcpy(snd,"GET /e.php?nmea=");
    strcat(snd,out_nmea);
    wifi.send(muxid, (const uint8_t*)snd, strlen(snd) );
    
    strcpy(snd," HTTP/1.1\r\n");
    strcat(snd,"Host: a.pajero.su\r\n");
    strcat(snd,"Connection: close\r\n\r\n");
    wifi.send(muxid, (const uint8_t*)snd, strlen(snd) );
    
    wifi.releaseTCP(muxid); 
}

// ---------------------- NMEA to Wi-Fi ----------------------------

void nmea_to_wifi( char char_nmea ) {

  char s[1];
  
  s[0] = char_nmea;

   uint8_t buffer[128] = {0};
   uint32_t len = 0;
  
 if (!wifi_connected) {
  len = wifi.recv(&mux_id, buffer, sizeof(buffer), 100);
  if (len > 0) wifi_connected = true;
 }
  
  if (wifi_connected) {
   if (!wifi.send(mux_id, (const uint8_t*)s, strlen(s) )) {
        wifi.releaseTCP(mux_id);
        wifi_connected = false;     
        len = 0;
   }
 }
}
