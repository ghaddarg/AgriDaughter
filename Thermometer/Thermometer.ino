//XXX: TODO: Add Lower Power Ability to wake up by button:
// https://learn.sparkfun.com/tutorials/reducing-arduino-power-consumption/all
// http://www.home-automation-community.com/arduino-low-power-how-to-run-atmega328p-for-a-year-on-coin-cell-battery/
// https://circuitdigest.com/microcontroller-projects/arduino-sleep-modes-and-how-to-use-them-to-reduce-power-consumption

//XXX: TODO: Add Solar abilities

/*
 * Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products
*/
// include the library code:
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <LowPower.h>

#define LCD_I2C_ADDRESS 0x27
LiquidCrystal_I2C lcd( LCD_I2C_ADDRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define DHTPIN 2        // Digital Pin 2 
#define DHTTYPE DHT11   // DHT 11
DHT dht( DHTPIN, DHTTYPE );

#define TEMP_MIN      4
#define TEMP_MAX      40
#define MOISTURE_MIN  20

const int analogInPin = A0;  // Analog input pin that the TMP36-1 sensor is attached to
int sensorValue       = 0;   
double temp           = 0;

#define MOISTURE_MIN  20
const int analogMsPin = A2;  // Analog input pin that the DHT11 sensor is attached to
int moistureVal       = 0;
double miosture       = 0;

int counter           = 0; //Sleep Counter

void setup( void ) 
{
  // put your setup code here, to run once:

  pinMode( LED_BUILTIN, OUTPUT );     // initialize digital pin LED_BUILTIN = 13 as an output.
  
#ifdef DEBUG
  Serial.begin( 9600 );              // initialize serial communications at 9600 bps:
#endif

  dht.begin();

  /* Set the LCD to a 20 chars and 4 line display */
  lcd.begin(20, 4);
  lcd.clear();
  /* Set cursor to position 0, 0 */ 
  lcd.home ();
  lcd.print("Hello, World LG!");
}

/* ***********************************************************************  */
/*                              TMP36-1 Sensor                              */
/* ***********************************************************************  */
void tmp_sensor( void )
{
  sensorValue = analogRead( analogInPin );  // Read the analog in value

  /* 
   * ADC conversion to Digital
   * Resolution of ADC / System Voltage = ADC steps Readings / Analog Voltage Measured
   *
   * So for 10-bits ADC conversion with 5V Max Voltage
   *  [ analogRead / 1023 ] = VoltageRead / 5V 
   */
  temp = ( double )sensorValue / 1024;    
  temp = temp * 5;

  /* 
   * TMP36-1 sensor conversion:
   * To convert the voltage to temperature, simply use the basic formula:
   * Temp in °C = [(Vout in mV) - 500] / 10
   */
   
  temp = temp - 0.5;                      
  temp = temp * 100;                     

#ifdef DEBUG
  Serial.print( "Analog value (mV) = " );   
  Serial.print( sensorValue );

  Serial.print( " - temp (C) = " );   
  Serial.println( temp );
#endif

  // print the number of seconds since reset:
  //lcd.print(millis() / 1000);

  /* Set cursor to position column 0, line 1 */ 
  /* (note: line 1 is the second row, since counting begins with 0) */
  lcd.setCursor( 0, 1 );
  lcd.print("Temp (C) = ");
  lcd.print(temp);

  if ( temp < TEMP_MIN ) {
    lcd.setCursor( 0, 0 );
    lcd.print("** TOO COLD **");
  } else if ( temp > TEMP_MAX ) {
    lcd.setCursor( 0, 0 );
    lcd.print("** TOO HOT **");
  }
}

/* ***********************************************************************  */
/*                                DHT11 Sensor                              */
/* ***********************************************************************  */
void hum_moist_sensor( void )
{
  moistureVal = analogRead( analogMsPin );  /* Read the analog in value */
  //Serial.print("moisture value  = " );                       
  //Serial.println(moistureVal);

  miosture = ( double )moistureVal / 1024;    
  miosture = miosture * 100;

  if ( miosture < MOISTURE_MIN ) {
    lcd.setCursor( 0, 0 );
    lcd.print("** Please Water Now **");
  }

  lcd.setCursor( 0, 2 );
  lcd.print("Moisture (%) = ");
  lcd.print( miosture );

#ifdef DEBUG
  Serial.print("Moisture (%)  = " );
  Serial.print( miosture );                      
#endif

   /* Reading temperature or humidity takes about 250 milliseconds! */
   /* Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor) */
  float h = dht.readHumidity();

  float t = dht.readTemperature();  /* Read temperature as Celsius (the default) */
  float f = dht.readTemperature( true ); /* Read temperature as Fahrenheit (isFahrenheit = true) */


  /* Check if any reads failed and exit early (to try again). */
  if (isnan(h) || isnan(t) || isnan(f)) {
#ifdef DEBUG
    Serial.println( "Failed to read from DHT sensor!" );
#endif

    lcd.setCursor( 0, 0 );
    lcd.print("Failed to read from DHT sensor! ");
    return;
  } 

  float hif = dht.computeHeatIndex(f, h); /* Compute heat index in Fahrenheit (the default) */
  float hic = dht.computeHeatIndex(t, h, false); /* Compute heat index in Celsius (isFahreheit = false) */

  lcd.setCursor( 0, 3 );
  lcd.print("Humidity (%) = ");
  lcd.print( h );
#ifdef DEBUG
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" % - ");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F - ");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
#endif
}

void blink_led( void )
{
  digitalWrite( LED_BUILTIN, HIGH );   
  delay( 1000 );
  digitalWrite(LED_BUILTIN, LOW); 
}

void loop( void ) 
{
  blink_led();

  tmp_sensor();

  delay( 2000 ); /*  DHT11 Sensor needs 2 sec before reading */

  hum_moist_sensor();

  blink_led();

  if ( counter < 10 ) {
    counter++;
  } else {
    lcd.off();
    
    /* ATmega328P, ATmega168 */
    LowPower.powerDown( SLEEP_FOREVER, ADC_OFF, BOD_OFF );
    //LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
  }
}
