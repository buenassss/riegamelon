/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 *
 * DESCRIPTION
 * Example sketch showing how to control physical relays.
 * This example will remember relay state after power failure.
 * http://www.mysensors.org/build/relay
 */

// Enable debug prints to serial monitor
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

//#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//#define ATC_RSSI      -80
#define IS_RFM69HW_HCW

#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>
#include <MySensors.h>

#define RELAY_PIN 3  // Arduino Digital I/O pin number for first relay (second on pin+1 etc) 3-7 digital pinout
#define NUMBER_OF_RELAYS 5 // Total number of attached relays
#define RELAY_ON 0  // GPIO value to write to turn on attached relay
#define RELAY_OFF 1 // GPIO value to write to turn off attached relay

#define TRIGGER_PIN  9  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
//#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define CHILD_ID 1   // Id of the sensor child

#define SLEEP_TIME            500 // Sleep time between reads (in milliseconds) (close to 2 hours)

bool pumpState[NUMBER_OF_RELAYS] = {false};
bool pumpNewState[NUMBER_OF_RELAYS] = {false};
bool ack = false;                                                     // set this to 1 if you want destination node to send ack back to this node
bool pumpChanged = false;

MyMessage pumpMsg;

//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
MyMessage sonarMsg(CHILD_ID + NUMBER_OF_RELAYS, V_DISTANCE);
int lastDist = -1;
// Constante velocidad sonido en cm/s
const float VelSon = 34000.0;

void setup()
{
  // Setup for relay
  for (int i = 0; i < NUMBER_OF_RELAYS; ++i)
  {
    int realPinRelay = RELAY_PIN + i;
    
    // Make sure relays are off when starting up
    digitalWrite(realPinRelay, RELAY_OFF);
    // Then set relay pins in output mode
    pinMode(realPinRelay, OUTPUT);   
  
    digitalWrite(realPinRelay, RELAY_OFF);
  }

  // Setup distance sensor
    // Ponemos el pin Trig en modo salida
  pinMode(TRIGGER_PIN, OUTPUT);
  // Ponemos el pin Echo en modo entrada
  pinMode(ECHO_PIN, INPUT);

}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("RiegaMelon", "1.0");

  for (int i = 0; i < NUMBER_OF_RELAYS; ++i)
  {
    // Register all sensors to gw (they will be created as child devices)
    present(CHILD_ID + i, S_BINARY, "Relay", ack);
  }
  
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID + NUMBER_OF_RELAYS, S_DISTANCE);
}

void initDistanceSensor()
{
  // Set Triger pint to low and wait 2 ms
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  
  // Set trigger to high and wait 10 ms
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  
  // Set trigger to low at the end
  digitalWrite(TRIGGER_PIN, LOW);
}

void loop()
{
  initDistanceSensor();
  
  // Get the time in ms that sensor needs to change state to HIGH with pulseIn
  unsigned long timeInMs = pulseIn(ECHO_PIN, HIGH);
  
  // Obtenemos la distancia en cm, hay que convertir el tiempo en segudos ya que está en microsegundos
  // por eso se multiplica por 0.000001
  int distanceInCm = timeInMs * 0.000001 * VelSon / 2.0;

  if (distanceInCm != lastDist) {
      send(sonarMsg.set(distanceInCm));
      lastDist = distanceInCm;

      Serial.print("Distance to water: ");
      Serial.print(distanceInCm); // Convert ping time to distance in cm and print result (0 = outside set distance range)
      Serial.println(" cm");
  }

  for (int i = 0; pumpChanged && i < NUMBER_OF_RELAYS; ++i)
  {
    if (pumpState[i] != pumpNewState[i])
    {
      pumpState[i] = pumpNewState[i];
      digitalWrite(RELAY_PIN + i, pumpState[i] ? RELAY_ON : RELAY_OFF);
      pumpMsg.setSensor(CHILD_ID + i);
      pumpMsg.setType(V_STATUS);
      send(pumpMsg.set(pumpState[i] ? true : false), true); // Send new state and request ack back
  
      Serial.print("Set relay ");
      Serial.print(RELAY_PIN + i);
      Serial.print(" to ");
      Serial.println(pumpState[i] ? "ON":" OFF");
  
    }
  }
  
  //sleep(SLEEP_TIME);
  //delay(SLEEP_TIME); // Not necessary because current resolution is about 400ms
}

void receive(const MyMessage &message)
{
  if (message.isAck())
  {
    Serial.println("This is an ack from gateway");
  }

  if (message.type == V_STATUS) {
    
     // Write some debug info
     Serial.print("Incoming change for sensor:");
     Serial.print(message.sensor-1);
     Serial.print(", New status: ");
     Serial.println(message.getBool());
     
     // Change relay state
     pumpNewState[message.sensor-1] = message.getBool();

     pumpChanged = true;
     
     // Store state in eeprom
     //saveState(CHILD_ID, state);
   } 
}
