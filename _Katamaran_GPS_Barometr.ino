// Arduino 1.0.6

#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <IRremote.h>
#include <Wire.h>
#include <ColorLCDShield.h>
#include <BMP085.h>


#define DISPLAY_1 16724175 // 1
#define DISPLAY_2 16718055 // 2
#define DISPLAY_3 16743045 // 3
#define DISPLAY_4 16716015 // 4
#define DISPLAY_5 16726215 // 5
#define DISPLAY_6 16734885 // 6
#define DISPLAY_7 16728765 // 7
#define DISPLAY_8 16730805 // 8
#define DISPLAY_9 16732845 // 9

#define CONTRAST_UP 16769055 // +
#define CONTRAST_DW 16754775 // -

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

//////////////////////////////////////////// Часы ////////////////////////

// Enter the time below in 12-hr format

#define HOURS 10
#define MINUTES 21
#define SECONDS 00
#define AMPM 0           // enter 0 for AM, 1 for PM

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

////////////////////////////////////////////

void setup()
{

  
  Wire.begin();
  delay(1000); // For BMP085
  set_1HZ_DS1307();
  
  // dps.init(); == dps.init(MODE_STANDARD, 0, true); 
  // dps.init(MODE_STANDARD, 101850, false);
  
  dps.init(MODE_ULTRA_HIGHRES, 25000, true); 
  
  irrecv.enableIRIn();
   
  pinMode(CPU_LED,OUTPUT);
  digitalWrite(CPU_LED,LOW);

  //pinMode(5,OUTPUT);
  //pinMode(7,OUTPUT);
  //pinMode(27,OUTPUT);
  //pinMode(26,OUTPUT);
  
  
  lcd.init(EPSON);   // Initializes lcd, using an PHILIPSdriver
  lcd.contrast(44);  // -51's usually a good contrast value
  lcd.clear(BLACK);  // clear the screen
  
  /*
  
  //Creates RED Arc in the ENE Quadrant with a Line Thickness of 5 Pixels
  int segmentsRed[1] = {ENE};
  lcd.setArc(60,50,40,segmentsRed,sizeof(segmentsRed),5,RED);
  
  //Creates YELLOW Arc in the NNE Quadrant with a Line Thickness of 10 Pixels
  int segmentsYellow[1] = {NNE};
  lcd.setArc(60,50,40,segmentsYellow,sizeof(segmentsYellow),10,YELLOW);
  
  //Creates GREEN Arc in the WNW and NNW Quadrants with a FILL
  int segments[2] = {WNW,NNW};
  lcd.setArc(60,50,40,segments,sizeof(segments),FILL,GREEN);
  
  //Creates PINK Circle with a FILL
  lcd.setCircle(90,100,20,PINK,FILL);
  
  //Creates CYAN Circle with a Line Thickness of 3 Pixels
  lcd.setCircle(90,35,25,CYAN,3);

*/

  Serial.begin(9600);
  Serial1.begin(4800);
  
  Serial.println("\nI2C Scanner");
  bt.begin(9600);


  ////////////////// Часы //////////////////////

  hours = HOURS;
  minutes = MINUTES;
  seconds = SECONDS;
  ampm = AMPM;
  
  drawClock();  // Draw the clock face, this includes 12, 3, 6, 9
  displayAnalogTime(hours, minutes, seconds);  // Draw the clock hands
  displayDigitalTime(hours, minutes, seconds, ampm);  // Draw the digital clock text
  
  ///////////////////////////////////////////////////////////////////////////////////////
  
}

void loop()
{
  
  
  
  
  /* We'll get here if it's been a second. We need to increase
  seconds by 1 and then go from there */
  seconds++;
  if (seconds >= 60)
  {
    seconds = 0;  // If seconds is 60, set it back to 0
    minutes++;    // and increase minutes by 1
    if (minutes >= 60)
    {
      minutes = 0;  // If minutes is 60, set it back to 0
      hours++;      // and increase hours by 1
      if (hours == 12)
        ampm ^= 1;  // If it's 12 o'clock, flip ampm
      if (hours >= 13)
        hours = 1;  // If hours is 13, set it to 1. 12-hr clock.
    }
  }
  /* Once each second, we'll redraw the clock with new values */
  drawClock();
  displayAnalogTime(hours, minutes, seconds);
  displayDigitalTime(hours, minutes, seconds, ampm);


  
  
  
    // i2scan();
  
   if (irrecv.decode(&results)) {
    bt.println(results.value);
    switch (results.value) {
    case 16724175: 
     lcd.off();
     break;
     case 16718055:
      lcd.on();
      break;
    }
    
    irrecv.resume(); 
   }     


 /*
   if (Serial1.available()) {
     char a = Serial1.read();
     bt.print(a);
   }   
   
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

void setTime()
{
  
}

/*
  displayDigitalTime() takes in values for hours, minutes, seconds
  and am/pm. It'll print the time, in digital format, on the
  bottom of the screen.
*/
void displayDigitalTime(int h, int m, int s, int ap)
{
  char timeChar[12];
  
  if (!ap)
  {
    sprintf(timeChar, "%.2d:%.2d:%.2d AM", h, m, s);
  }
  else
  {
    sprintf(timeChar, "%.2d:%.2d:%.2d PM", h, m, s);
  }
  /* Print the time on the clock */
  lcd.setStr(timeChar, CLOCK_CENTER + CLOCK_RADIUS + 4, 22, 
              C_COLOR, BACKGROUND);
}

/*
  drawClock() simply draws the outer circle of the clock, and '12',
  '3', '6', and '9'. Room for growth here, if you want to customize
  your clock. Maybe add dashe marks, or even all 12 digits.
*/
void drawClock()
{
  /* Draw the circle */
  lcd.setCircle(CLOCK_CENTER, 66, CLOCK_RADIUS, C_COLOR);
  
  /* Print 12, 3, 6, 9, a lot of arbitrary values are used here
     for the coordinates. Just used trial and error to get them 
     into a nice position. */
  lcd.setStr("12", CLOCK_CENTER - CLOCK_RADIUS, 66-9, C_COLOR, BACKGROUND);
  lcd.setStr("3", CLOCK_CENTER - 9, 66 + CLOCK_RADIUS - 12, C_COLOR, BACKGROUND);
  lcd.setStr("6", CLOCK_CENTER + CLOCK_RADIUS - 18, 66-4, C_COLOR, BACKGROUND);
  lcd.setStr("9", CLOCK_CENTER - 9, 66 - CLOCK_RADIUS + 4, C_COLOR, BACKGROUND);
}

/*
  displayAnalogTime() draws the three clock hands in their proper
  position. Room for growth here, I'd like to make the clock hands
  arrow shaped, or at least thicker and more visible.
*/
void displayAnalogTime(int h, int m, int s)
{
  double midHours;  // this will be used to slightly adjust the hour hand
  static int hx, hy, mx, my, sx, sy;
  
  /* Adjust time to shift display 90 degrees ccw
     this will turn the clock the same direction as text */
  h -= 3;
  m -= 15;
  s -= 15;
  if (h <= 0)
    h += 12;
  if (m < 0)
    m += 60;
  if (s < 0)
    s += 60;
    
  /* Delete old lines: */
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+sx, 66+sy, BACKGROUND);  // delete second hand
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+mx, 66+my, BACKGROUND);  // delete minute hand
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+hx, 66+hy, BACKGROUND);  // delete hour hand
  
  /* Calculate and draw new lines: */
  s = map(s, 0, 60, 0, 360);  // map the 0-60, to "360 degrees"
  sx = S_LENGTH * sin(3.14 * ((double) s)/180);  // woo trig!
  sy = S_LENGTH * cos(3.14 * ((double) s)/180);  // woo trig!
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+sx, 66+sy, S_COLOR);  // print second hand
  
  m = map(m, 0, 60, 0, 360);  // map the 0-60, to "360 degrees"
  mx = M_LENGTH * sin(3.14 * ((double) m)/180);  // woo trig!
  my = M_LENGTH * cos(3.14 * ((double) m)/180);  // woo trig!
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+mx, 66+my, M_COLOR);  // print minute hand
  
  midHours = minutes/12;  // midHours is used to set the hours hand to middling levels between whole hours
  h *= 5;  // Get hours and midhours to the same scale
  h += midHours;  // add hours and midhours
  h = map(h, 0, 60, 0, 360);  // map the 0-60, to "360 degrees"
  hx = H_LENGTH * sin(3.14 * ((double) h)/180);  // woo trig!
  hy = H_LENGTH * cos(3.14 * ((double) h)/180);  // woo trig!
  lcd.setLine(CLOCK_CENTER, 66, CLOCK_CENTER+hx, 66+hy, H_COLOR);  // print hour hand
}
