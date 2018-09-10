#include <TimerOne.h>

// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RF24_PA_LEVEL RF24_PA_LOW

//#define MY_NODE_ID 2  // Set this to fix your Radio ID or use Auto
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
  ledState = !ledState ;
}

void setup() {
  // put your setup code here, to run once:
  // initialize serial communication at 9600 bits per second:
  //Serial.begin(115200);

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

  delay(5000);
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

