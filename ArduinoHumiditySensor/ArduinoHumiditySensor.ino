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

#include <SPI.h>
#include <MySensors.h>


// Constants
#define N_ELEMENTS(array) (sizeof(array)/sizeof((array)[0]))
#define THRESHOLD             1.1     // Only make a new reading with reverse polarity if the change is larger than 10%
#define STABILIZATION_TIME    1000    // Let the sensor stabilize before reading
const int SENSOR_ANALOG_PINS[] = {A0, A1};
#define CHILD_ID_MOISTURE     0

// Global variables
byte direction = 0;
int oldMoistureLevel = -1;
int ledState = LOW;

MyMessage msgMoisture(CHILD_ID_MOISTURE, V_HUM);


void presentation()  
{ 
  sendSketchInfo("RiegaMelon", "1.0");

  //Register sensor
  present(CHILD_ID_MOISTURE, S_HUM);
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
  Serial.print(humidity);
  Serial.println("%");
  Serial.flush();

  send(msgMoisture.set(humidity, 2));

  delay(1000);
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

