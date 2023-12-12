#include <LiquidCrystal_I2C.h>
#include <Wire.h> //This is for i2C
#include <AccelStepper.h>
#include <Arduino_FreeRTOS.h>

LiquidCrystal_I2C lcd(0x27,A4,A5);  // set the LCD address to 0x3F for a 16 chars and 2 line display

// Definir pines para el motor paso a paso
const int stepPin = 10;
const int dirPin = 11;


// Configurar el objeto stepper
AccelStepper stepper1(AccelStepper::FULL4WIRE, stepPin, dirPin);

int ThermistorPin = 0;
int Vo;
float R1 = 10000;
const int RELAY_PIN = 3;  // the Arduino pin, which connects to the IN pin of relay
float logR2, R2, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

//Variables
float temperature_read = 0.0;
char templcd[12];
char vellcd[12];
  
float set_temperature = 250; //valor de setpoint

float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;

//PID constants
int kp = 6;   int ki = 0.3;   
// int kd = 1.8;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;

//Magnetic sensor things
int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber, previousquadrantNumber; //quadrant IDs
float numberofTurns = 0; //number of turns
float correctedAngle = 0; //tared angle - based on the startup value
float startAngle = 0; //starting angle
float totalAngle = 0; //total absolute angular displacement
float previoustotalAngle = 0; //for the display printing




void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  Time = millis(); 

  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
  // Configurar el motor paso a paso
  stepper1.setMaxSpeed(1000);
  stepper1.setSpeed(0);
}

void loop() {
  noInterrupts();
  stepper1.setSpeed(100);   // Establecer velocidad del motor paso a paso
  stepper1.runSpeed();
  interrupts();
  temperature_read = readThermocouple();
  // error en estado de trancisión
  PID_error = set_temperature - temperature_read;
  // constante proporcional
  PID_p = kp * PID_error;
  // Calcular el valor de integración
  if(-20 < PID_error < 20)
  {
    PID_i = PID_i + (ki * PID_error);  
  }
  timePrev = Time;
  Time = millis();
  //tiempo de coneccion
   elapsedTime = (Time - timePrev) / 1000; 
   // Calcular valor de derivaciónd
   //PID_d = kd*((PID_error - previous_error)/elapsedTime);
   //Valor de la suma de P + I + D
   PID_value = PID_p + PID_i;
   if(PID_value < 0)
    {    PID_value = 0;    }
   if(PID_value > 255)  
    {    PID_value = 1;  }
    digitalWrite(RELAY_PIN, PID_value);
    previous_error = PID_error;     //Guarda el error 
    Serial.print(temperature_read);
    Serial.println(" C");  
    //lcd.clear();
    lcd.setCursor(1,0);   //Set cursor to character 2 on line 0
    lcd.print("Temp:");
    lcd.setCursor(10,0);
    lcd.print(temperature_read);
    lcd.setCursor(14,0);   //Set cursor to character 2 on line 0
    lcd.print(" C");
  
    lcd.setCursor(1,1);   //Move cursor to character 2 on line 1
    lcd.print("Vel:");
  
    lcd.setCursor(10,1);
    lcd.print(1);
  
    lcd.setCursor(12,1);   //Set cursor to character 2 on line 0
    lcd.print("mm/s");
    
    //delay(300);
    
}

double readThermocouple() {
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15 ;
  return Tc;
}


void update_screen(int temp, int vel)
{
  lcd.clear();  
  // Print a message on both lines of the LCD.
  lcd.setCursor(1,0);   //Set cursor to character 2 on line 0
  lcd.print("Temp:");
  lcd.setCursor(10,0);
  lcd.print(sprintf(templcd, "%d", temp));
  lcd.setCursor(14,0);   //Set cursor to character 2 on line 0
  lcd.print(" C");
  
  lcd.setCursor(1,1);   //Move cursor to character 2 on line 1
  lcd.print("Vel:");
  
  lcd.setCursor(10,1);
  lcd.print(sprintf(vellcd, "%d", vel));
  
  lcd.setCursor(12,1);   //Set cursor to character 2 on line 0
  lcd.print("mm/s");
  
}
