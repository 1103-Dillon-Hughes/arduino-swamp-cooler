//Inlude necessary libraries
#include <AccelStepper.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <Wire.h>
#include <RTClib.h>
#include <time.h>

#define RDA 0x80
#define TBE 0x20  

// Define stepper motor connections
#define motorPin1 23
#define motorPin2 25
#define motorPin3 27
#define motorPin4 29

#define DHTPIN 45          // Pin connected to DHT11 sensor
#define DHTTYPE DHT11     // DHT11 sensor type
DHT dht(DHTPIN, DHTTYPE); // Create DHT object

//Serial Registers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

//ADC registers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//Pin B registers
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
//Pin H registers
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h  = (unsigned char*) 0x101;
volatile unsigned char* pin_h  = (unsigned char*) 0x100;

//Timer interrupt registers
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;


RTC_DS1307 RTC;

// LCD pins
LiquidCrystal lcd(53, 52, 51, 50, 49, 48);

void setup() 
{
  // put your setup code here, to run once:
  lcd.begin(16, 2);  // Initialize LCD
  dht.begin();       // Initialize DHT sensor

  //Set Baud rate  
  U0init(9600);  

  //Initialize ADC
  adc_init();

  //RTC Setup
  Wire.begin();
  RTC.begin();
  RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));

  //LED pins
  pinModeB(7,OUTPUT);
  pinModeB(6,OUTPUT);
  pinModeB(5,OUTPUT);
  pinModeB(4,OUTPUT);

  //Button pins
  pinModeH(6, INPUT);
  pinModeH(5, INPUT);
  pinModeH(4, INPUT);

  //Motor Control pin
  pinModeH(3, OUTPUT);
 
}

bool idleTimestamp = false;
bool runningTimestamp = false;

void loop() {

  //If the reset button or the temperature is above the threshold in degrees C, enter idle state
  if(digitalReadH(5) || dht.readTemperature() <= 20) 
  {
    idleState();

    // Print timestamp if it hasn't been printed yet
    if(!idleTimestamp)
    {
      U0putstr("Idle State: ");
      printTimestamp();
      idleTimestamp = true;
    }


    //If disable button is pressed, enter disabled state, wait till button is pressed again
    //Delay for debouncing
    if(digitalReadH(4)) 
    { 
      //Print timestamp to serial monitor
      U0putstr("Disabled State: ");
      printTimestamp();

      myDelay(1000);
      while(!digitalReadH(4))
      {
      disabledState(); 
      myDelay(600);      
      }
      myDelay(1000);

      //Reset Idle timestamp variable
      idleTimestamp = false;
    }

    //If the water level drops below the threshold, enter error state
    if(adc_read(0) <= 200) 
    {
      //Print timestamp to serial monitor
      U0putstr("Error State: ");
      printTimestamp();
      
      //Display error message to LCD      
      displayError();
      //Stay in error state until reset button is pressed
      while(!digitalReadH(5))
      {
        errorState();

        //Move vent position when vent button pressed      
        if(digitalReadH(6))
          {
            spinMotor();    
          }   
          
      }
      //Reset Idle timestamp variable
      idleTimestamp = false;
    } 

    //Move vent position when vent button pressed
    if(digitalReadH(6))
    {
        spinMotor();     
    }
  } 

  //If temp above threshold in C, enter running state
  else if(dht.readTemperature() > 20) 
  {
    runningState();

    // Print timestamp if it hasn't been printed yet
    if(!runningTimestamp)
    {
      U0putstr("Running State: ");
      printTimestamp();
      runningTimestamp = true;
    }
    
    //If disable button is pressed, enter disabled state, wait till button is pressed again
    //Delay for debouncing
    if(digitalReadH(4)) 
    {
      //Print timestamp to serial monitor
      U0putstr("Disabled State: ");
      printTimestamp();

      myDelay(1000);
      while(!digitalReadH(4))
      {
        disabledState();
        myDelay(600);
      }
      myDelay(1000);

      //Reset Running timestamp variable
      runningTimestamp = false;
      //Reset Idle timestamp variable
      idleTimestamp = false;      
    }
    
    //If water falls below threshold, enter error state
    if(adc_read(0) <= 200) 
    {
      //Print timestamp to serial monitor
      U0putstr("Error State: ");
      printTimestamp();      

      //Display error message to LCD
      displayError();

      //Stay in error state until reset button is pressed
      while(!digitalReadH(5))
      {
        errorState();
        //Move vent position when vent button pressed
        if(digitalReadH(6))
          {
            spinMotor();    
          }    
      }
      //Reset Running timestamp variable
      runningTimestamp = false;
      //Reset Idle timestamp variable
      idleTimestamp = false;      
    }  
    
    //Move vent position when vent button pressed
    if(digitalReadH(6))
    {
      spinMotor();    
    }    
  } 
}


// Initialize the stepper motor object
AccelStepper stepper(AccelStepper::FULL4WIRE, motorPin1, motorPin2, motorPin3, motorPin4);

// Spin motor in the clockwise direction
void spinMotor() 
{
  // Set the maximum speed and acceleration of the stepper motor
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // Spin the stepper motor continuously in the clockwise direction
  stepper.moveTo(200);
  stepper.runToPosition();
  stepper.moveTo(0);
  stepper.runToPosition();

}



//Measure the temp and humidity and display it on LCD
void displayTemperatureHumidity() 
{
  // Read temperature and humidity values from DHT11 sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Display temperature and humidity values on LCD screen
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print(" %");
}

// Function to display error message on LCD screen
void displayError() 
{
  // Clear LCD screen
  lcd.clear();
  
  lcd.setCursor(0, 0);
  lcd.print("ERROR:");
  lcd.setCursor(0, 2);
  lcd.print("WATER IS LOW!");
}

//Set port b pins to either input or output
void pinModeB(unsigned char pin_num, unsigned char mode)
{
    if(mode == OUTPUT)
    {
        *ddr_b |= 0x01 << pin_num;
    }
    else
    {
        *ddr_b &= ~(0x01 << pin_num);
    }
}

//Write port b pins to either high or low
void digitalWriteB(unsigned char pin_num, unsigned char state)
{
    if(state == 0)
    {
        *port_b &= ~(0x01 << pin_num);
    }
    else
    {
        *port_b |= 0x01 << pin_num;
    }
}

//Set port h pins to either input or output
void pinModeH(unsigned char pin_num, unsigned char mode)
{
    if(mode == OUTPUT)
    {
        *ddr_h |= 0x01 << pin_num;
    }
    else
    {
        *ddr_h &= ~(0x01 << pin_num);
    }
}

//Write port h pins to either high or low
void digitalWriteH(unsigned char pin_num, unsigned char state)
{
    if(state == 0)
    {
        *port_h &= ~(0x01 << pin_num);
    }
    else
    {
        *port_h |= 0x01 << pin_num;
    }
}

//Check if port h pins are either high or low
bool digitalReadH(unsigned char pin) {
  // Check the state of the specified pin
  return (*pin_h & (1 << pin));
}

//Send timestamp to serial monitor
void printTimestamp() 
{
  time_t now = RTC.now().unixtime();
  char time_str[20];
  strftime(time_str, 20, "%Y-%m-%d %H:%M:%S", localtime(&now));

  U0putstr(time_str);
  U0putstr("\r\n");
}

//Initialize ADC
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

//Read value from specified pin
unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

//Initialize serial monitor
void U0init(unsigned long U0baud) 
{
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

unsigned char U0kbhit() 
{
  return (*myUCSR0A & RDA);
}

//Read character
unsigned char U0getchar() 
{
  while (U0kbhit() == 0){}; // wait for RDA = true
  return *myUDR0;
}


//Write character
void U0putchar(unsigned char U0pdata) 
{
  while ((*myUCSR0A & TBE) == 0){}; // wait for TBE = true
  *myUDR0 = U0pdata;
}

//Write string
void U0putstr(char *s) 
{
  while (*s) {
    U0putchar(*s++);
  }
}

void disabledState()
{
  //Drive yellow LED HIGH
  digitalWriteB(6,HIGH);
  //Drive all other LEDs LOW
  digitalWriteB(7,LOW);
  digitalWriteB(5,LOW);
  digitalWriteB(4,LOW);  

  //Turn off DC Motor
  digitalWriteH(3, LOW);

  //Clear LCD
  lcd.clear();

  // Display disabled on LCD screen
  lcd.setCursor(0, 0);
  lcd.print("DISABLED");

}

void idleState()
{
  //Drive green LED HIGH
  digitalWriteB(5,HIGH);
  //Drive all other LEDs LOW
  digitalWriteB(7,LOW);
  digitalWriteB(6,LOW);
  digitalWriteB(4,LOW);

  //Turn off DC Motor
  digitalWriteH(3, LOW);
  
  //Display temp and humidity to LCD screen
  displayTemperatureHumidity();
}

void runningState()
{
  //Drive blue LED HIGH
  digitalWriteB(4,HIGH);
  //Drive all other LEDs LOW
  digitalWriteB(7,LOW);
  digitalWriteB(5,LOW);
  digitalWriteB(6,LOW);

  //Turn on DC Motor
  digitalWriteH(3, HIGH);

  //Display temp and humidity to LCD screen  
  displayTemperatureHumidity();

}

void errorState()
{
  //Drive red LED HIGH
  digitalWriteB(7,HIGH);
  //Drive all other LEDs LOW
  digitalWriteB(6,LOW);
  digitalWriteB(5,LOW);
  digitalWriteB(4,LOW);

  //Turn off DC Motor
  digitalWriteH(3, LOW);
}

void myDelay(unsigned long ms) {
  // calculate the number of ticks required for the delay
  unsigned long ticks = (ms * 1000) / 62.5;
  // loop to repeatedly call my_delay() until the desired delay time has passed
  for (unsigned long i = 0; i < ticks; i++) {
    // stop the timer
    *myTCCR1B &= 0xF8;
    // set the counts
    *myTCNT1 = (unsigned int) (65536 - 30000);
    // start the timer
    *myTCCR1B |= 0b00000001;
    // wait for overflow
    while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
    // stop the timer
    *myTCCR1B &= 0xF8;   // 0b 0000 0000
    // reset TOV           
    *myTIFR1 |= 0x01;
  }
}