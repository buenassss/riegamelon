#include <TimerOne.h>

// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_RFM69  // Define for using RFM69 radio
//#define MY_RFM69_FREQUENCY RFM69_868MHZ  // Define for frequency setting. Needed if you're radio module isn't 868Mhz (868Mhz is default in lib)
#define MY_IS_RFM69HW  // Mandatory if you radio module is the high power version (RFM69HW and RFM69HCW), Comment it if it's not the case
//#define MY_RFM69_NETWORKID 100  // Default is 100 in lib. Uncomment it and set your preferred network id if needed
//#define RFM69_IRQ_PIN 4  // Default in lib is using D2 for common Atmel 328p (mini pro, nano, uno etc.). Uncomment it and set the pin you're using. Note for Atmel 328p, Mysensors, and regarding Arduino core implementation D2 or D3 are only available. But for advanced mcus like Atmel SAMD (Arduino Zero etc.), Esp8266 you will need to set this define for the corresponding pin used for IRQ
// #define MY_RFM69_IRQ_NUM 4 // Temporary define (will be removed in next radio driver revision). Needed if you want to change the IRQ pin your radio is connected. So, if your radio is connected to D3/INT1, value is 1 (INT1). For others mcu like Atmel SAMD, Esp8266, value is simply the same as your RFM69_IRQ_PIN
// #define MY_RFM69_SPI_CS 15 // If using a different CS pin for the SPI bus. Use MY_RFM69_CS_PIN for the development branch.
#define MY_RFM69_NEW_DRIVER

#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>
#include <MySensors.h>

// Constants
#define N_ELEMENTS(array) (sizeof(array)/sizeof((array)[0]))
#define THRESHOLD             1.1     // Only make a new reading with reverse polarity if the change is larger than 10%
#define STABILIZATION_TIME    1000    // Let the sensor stabilize before reading
const int SENSOR_ANALOG_PINS[] = {A0, A1};
#define CHILD_ID_MOISTURE     0

//Battery voltage
#define BATTERY_FULL          4200    // litio usually gives 3.7V when full (3329 is the max that arduino accept due to regulator)
#define BATTERY_ZERO          2340    // 2.34V limit for 328p at 8MHz
const int BATTERY_PIN   =  A2;
const long R1 = 10000;   //ohms of R1
const float toleranceR1 = 0.02; // Resistor tolerance
const long R2 = 10000;   // ohms of R2
const float toleranceR2 = 0.02; // Resistor tolerance
const float maxVoltage = 3.3; // voltage of Arduino (5, 3.3)
const float toleranceADC = 0.0; // ADC tolerance


#define CHILD_ID_VOLTAGE      1

MyMessage voltageMsg(CHILD_ID_VOLTAGE, V_VOLTAGE);  // Node voltage

#define SLEEP_TIME            7200000 // Sleep time between reads (in milliseconds) (close to 2 hours)

// Global variables
byte direction = 0;
int oldMoistureLevel = -1;
int ledState = LOW;

MyMessage msgMoisture(CHILD_ID_MOISTURE, V_HUM);
//MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE);

#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
#define IS_RFM69HW_HCW

void presentation()  
{
  sendSketchInfo("RiegaMelon", "1.0");

  //Register sensor
  present(CHILD_ID_MOISTURE, S_HUM);
  present(CHILD_ID_VOLTAGE, S_MULTIMETER, "Battery " );


}

// Interruptions
void ISR_Blink()
{
  ledState = (ledState == LOW ? HIGH : LOW) ;
}

void setup() {
  // put your setup code here, to run once:
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Timer1.initialize(1000000);         // Dispara cada 1s
  Timer1.attachInterrupt(ISR_Blink); // Activa la interrupcion y la asocia a ISR_Blink  

  //Set moisutre sensor pins
  for (int i = 0; i < N_ELEMENTS(SENSOR_ANALOG_PINS); i++)
  {
    pinMode(SENSOR_ANALOG_PINS[i], OUTPUT);
    digitalWrite(SENSOR_ANALOG_PINS[i], LOW);
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, ledState);

  //Get moisture level
  int moistureLevel = readMoisture();
   
  //Send rolling average of 2 samples to get rid of the "ripple" produced by different resistance in the internal pull-up resistors
  //See http://forum.mysensors.org/topic/2147/office-plant-monitoring/55 for more information
  //Check if it was first reading, save current value as old
  if (oldMoistureLevel == -1)
  { 
    oldMoistureLevel = moistureLevel;
  }

  //Verify if current measurement is not too far from the previous one
  if (moistureLevel > (oldMoistureLevel * THRESHOLD) || moistureLevel < (oldMoistureLevel / THRESHOLD))
  {
    //The change was large, so it was probably not caused by the difference in internal pull-ups.
    //Measure again, this time with reversed polarity.
    moistureLevel = readMoisture();
  }

  //Store current moisture level
  oldMoistureLevel = moistureLevel;
  float humidity = (moistureLevel + oldMoistureLevel) / 2.0 / 10.23;
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  Serial.flush();

  send(msgMoisture.set(humidity, 2));

  //Report data to the gateway
  long voltage = getVoltageFromPin(BATTERY_PIN);
  Serial.print("voltage: ");
  Serial.println(voltage);
  
  int batteryPcnt = round((voltage - BATTERY_ZERO) * 100.0 / (BATTERY_FULL - BATTERY_ZERO));
  if (batteryPcnt > 100) 
  {
    batteryPcnt = 100;
  } 
  else if (batteryPcnt < 0)
  {
    batteryPcnt = 0;
  }
  sendBatteryLevel(batteryPcnt);
  send(voltageMsg.set((float)(voltage)/1000,2));
  
  Serial.print("battery: ");
  Serial.print(batteryPcnt);
  Serial.println("%");
  Serial.flush();
  
//  delay(1000);
  sleep(SLEEP_TIME);

}

/**************************************************************************************/
/* Allows to get moisture.                                                            */
/**************************************************************************************/
int readMoisture()
{
  //Power on the sensor and read once to let the ADC capacitor start charging
  pinMode(SENSOR_ANALOG_PINS[direction], INPUT_PULLUP);
  analogRead(SENSOR_ANALOG_PINS[direction]);
  
  int moistureLevel = (1023 - analogRead(SENSOR_ANALOG_PINS[direction]));

  //Turn off the sensor to conserve battery and minimize corrosion
  pinMode(SENSOR_ANALOG_PINS[direction], OUTPUT);
  digitalWrite(SENSOR_ANALOG_PINS[direction], LOW);

  //Make direction alternate between 0 and 1 to reverse polarity which reduces corrosion
  direction = (direction + 1) % 2;
  return moistureLevel;
}

/**************************************************************************************/
/* Allows to get the real Vcc (return value in mV).                                   */
/* http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/   */
/**************************************************************************************/
long getVoltage()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

// Vcc in millivolts
long getVoltageFromPin(int analogPin)
{
  float voltageInPin = analogRead(analogPin) * maxVoltage / 1024;

  Serial.print("Voltage in pin: ");
  Serial.print(analogPin);
  Serial.print(": ");
  Serial.print(analogRead(analogPin));
  Serial.print(" --> ");
  Serial.print(voltageInPin);
  Serial.print("V - ");

  long realR1 = R1 + R1 * toleranceR1;
  long realR2 = R2 + R2 * toleranceR2;

  // Applying tension divisor formula Vout = (R2/(R1+R2))*Vin
  float vIn = voltageInPin * (realR1 + realR2) / realR2;

  Serial.println(vIn+ vIn * toleranceADC);

  return (vIn + vIn * toleranceADC)*1000;
}

