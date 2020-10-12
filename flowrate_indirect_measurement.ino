/* Serial Data Aquisition for Pressure Sensors & Fan Control
   Included Sensors:
   - Huba
*/

//Libraries-----------------------------------------------------------------------------------------

#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <FreqCount.h>

//Constants-----------------------------------------------------------------------------------------

//Pressure Sensor-related constants
#define HUBA_PIN_01 A0 
#define HUBA_PIN_02 A1

//Recording indicator signal constants
#define CONTROL_PIN 8

// PWM Encoder
#define PWM_ENCODER A3

// Serial Output-related constants:
#define N_sensors 6

// Duty Cycle Measurement
#define pulse_ip 10

//Initialize classes---------------------------------------------------------------------------------

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

//Variables------------------------------------------------------------------------------------------
float x1,x2,control,n_measurement, encoder, period , duty ;
int ontime,offtime;

//Serial Output-related variables
int serial_address[N_sensors];
int pwm_control_value;
double buffer_sensordata[N_sensors];

//LCD Output-related variables
String display_name[N_sensors] = {
  "REC",
  "P_in",
  "P_out",
  "Freq",
  "PWM",
  "Duty"
};
String display_units[N_sensors] = {
  "[-]",
  "[Pa]",
  "[Pa]",
  "[Hz]",
  "[%]",
  "[%]"
};
int lcd_visualization_group[4] = {
  0,
  1,
  5,
  3,
};

//Functions-----------------------------------------------------------------------------------------

void analogWrite25k(int pin, int value)
{
  switch (pin) {
    case 9:
      OCR1A = value;
      break;
    case 10:
      OCR1B = value;
      break;
    default:
      // no other pin will work
      break;
  }
}

void setup_pwm_frequency()
{
    // Configure Timer 1 for PWM @ 25 kHz.
  TCCR1A = 0;           // undo the configuration done by...
  TCCR1B = 0;           // ...the Arduino core library
  TCNT1  = 0;           // reset timer
  TCCR1A = _BV(COM1A1)  // non-inverted PWM on ch. A
           | _BV(COM1B1)  // same on ch; B
           | _BV(WGM11);  // mode 10: ph. correct PWM, TOP = ICR1
  TCCR1B = _BV(WGM13)   // ditto
           | _BV(CS10);   // prescaler = 1
  ICR1   = 320;         // TOP = 320
}


//Set-up--------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  setup_pwm_frequency();
  FreqCount.begin(1000);
  

  pinMode(8, INPUT);
  // Set the PWM pins as output.
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(pulse_ip,INPUT);


}

//Loop----------------------------------------------------------------------------------------------

void loop()
{
  // Re-initialize data buffer to 0
  for (int x = 0; x < N_sensors; x = x + 1) {
    serial_address[x] = x + 1;
    buffer_sensordata[x] = 0.00;
  }


  //Store sensor data in data buffer
  x1 = analogRead(HUBA_PIN_01);
  x2 = analogRead(HUBA_PIN_02);
  n_measurement = FreqCount.read();
  encoder = analogRead(PWM_ENCODER)/10;
  
  ontime = pulseIn(pulse_ip,HIGH);
  offtime = pulseIn(pulse_ip,LOW);
  period = ontime+offtime;
  duty = (ontime/period)*100;

    if (encoder > 100){
    encoder = 100;
  }
  else if (encoder < 0){
    encoder = 0;
  };

  
  control=digitalRead(CONTROL_PIN);
  buffer_sensordata[0]=control;
  buffer_sensordata[1] = ((x1 / 1023) * 5) * 200 - 100; //Huba Characteristic
  buffer_sensordata[2] = ((x2 / 1023) * 5) * 200 - 100; //Huba Characteristic
  buffer_sensordata[3] = n_measurement;
  buffer_sensordata[4] = encoder;
  buffer_sensordata[5] = duty;

  
  //Serial Output
  for (int n = 0; n < N_sensors; n = n + 1) {
    Serial.print(serial_address[n]);
    Serial.print(": ");
    Serial.print(buffer_sensordata[n]);
    Serial.print(" ,");
  }
  Serial.println("");

  //LCD Output
  lcd.clear();
  for (int n2 = 0; n2 < 4; n2 = n2 + 1) {
    lcd.setCursor(0, n2);
    lcd.print(display_name[lcd_visualization_group[n2]] + ": ");
    lcd.print(String(buffer_sensordata[lcd_visualization_group[n2]]));
    lcd.print(" ");
    lcd.print(display_units[lcd_visualization_group[n2]]);
  };

  delay(250);
}
