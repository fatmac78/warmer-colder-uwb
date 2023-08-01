/*

For ESP32 UWB or ESP32 UWB Pro

*/

#include <SPI.h>
#include "DW1000Ranging.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// leds pins
const uint8_t PIN_COLD1 = 33; // warmer level 1 LED pin
const uint8_t PIN_COLD2 = 36; // warmer level 2 LED pin
const uint8_t PIN_COLD3 = 39; // warmer level 3 LED pin
const uint8_t PIN_WARM1 = 2; // colder level 1 LED pin     
const uint8_t PIN_WARM2 = 12; // colder level 2 LED pin     
const uint8_t PIN_WARM3 = 13; // colder level 3 LED pin     
const uint8_t PIN_DETECT = 25; // hidden node detected LED    
const uint8_t PIN_NEAR1 = 22; // warmer level 1 LED pin
const uint8_t PIN_NEAR2 = 21; // warmer level 1 LED pin
const uint8_t PIN_NEAR3 = 26; // warmer level 1 LED pin
const uint8_t PIN_NEAR4 = 15; // warmer level 1 LED pin

// trigger thresholds
const double WARM_THRESHOLD_1 = -0.1;
const double WARM_THRESHOLD_2 = -0.3;
const double WARM_THRESHOLD_3 = -0.6;
const double COLD_THRESHOLD_1 = 0.1;
const double COLD_THRESHOLD_2 = 0.3;
const double COLD_THRESHOLD_3 = 0.6;
const double PACKET_THRESHOLD = 0.7;

static double distance;
static double filtered_delta;

// global store distances
static double new_distance;
static double distance_n_minus_1;
static double distance_n_minus_2;
static double distance_n_minus_3;
static double distance_n_minus_4;
static double distance_n_minus_5;

// global store pings
static double filtered_ping;
static int new_active_ping = 0;
static int active_ping_n_minus_1 = 0;
static int active_ping_n_minus_2 = 0;
static int active_ping_n_minus_3 = 0;
static int active_ping_n_minus_4 = 0;
static int active_ping_n_minus_5 = 0;

void setup()
{
    Serial.begin(115200);

    pinMode(PIN_WARM1, OUTPUT);
    pinMode(PIN_WARM2, OUTPUT);
    pinMode(PIN_WARM3, OUTPUT);
    pinMode(PIN_COLD1, OUTPUT);
    pinMode(PIN_COLD2, OUTPUT);
    pinMode(PIN_COLD3, OUTPUT);
    pinMode(PIN_DETECT, OUTPUT);
    pinMode(PIN_NEAR1, OUTPUT);
    pinMode(PIN_NEAR2, OUTPUT);
    pinMode(PIN_NEAR3, OUTPUT);
    pinMode(PIN_NEAR4, OUTPUT);
    
    delay(1000);
    //init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
    //define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    //Enable the filter to smooth the distance
    //DW1000Ranging.useRangeFilter(true);

    //we start the module as a tag
    DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9D", DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
}

void loop()
{
    DW1000Ranging.loop();
   
    
}

void newRange()
{
    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm TAG");

    new_distance = DW1000Ranging.getDistantDevice()->getRange();

    new_active_ping = 1;
    filtered_delta=filtered_delta_distance(new_distance, distance_n_minus_1, distance_n_minus_2, distance_n_minus_3, distance_n_minus_4, distance_n_minus_5);
    Serial.println(filtered_delta);
    
    update_temperature_indicators(filtered_delta);
    shift_values();

    if (new_distance > 1.0){
        Serial.println("Near 1");
        digitalWrite(PIN_NEAR1, HIGH);  // turn the LED off
    } 
    else {
        digitalWrite(PIN_NEAR1, LOW);  // turn the LED off
    }
    if (new_distance > 2.0){
        Serial.println("Near 2");
        digitalWrite(PIN_NEAR2, HIGH);  // turn the LED off
    } else {
        digitalWrite(PIN_NEAR2, LOW);  // turn the LED off
    }
    if (new_distance > 3.0){
        Serial.println("Near 3");
        digitalWrite(PIN_NEAR3, HIGH);  // turn the LED off
    } else {
        digitalWrite(PIN_NEAR3, LOW);  // turn the LED off
    }
    if (new_distance > 4.0){
        Serial.println("Near 4");
        digitalWrite(PIN_NEAR4, HIGH);  // turn the LED off
    } else {
        digitalWrite(PIN_NEAR4, LOW);  // turn the LED off
    }

    delay(250);

    
}

void newDevice(DW1000Device *device)
{
    Serial.print("ranging init; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}

void update_detected_indicator(double filtered_ping){
     
     if (filtered_ping > PACKET_THRESHOLD){
         digitalWrite(PIN_DETECT, HIGH); // turn the LED on
     } 
     else {
         digitalWrite(PIN_DETECT, LOW); // turn the LED on
         Serial.println("Dropped packets");
     }
}

void update_temperature_indicators(double filtered_delta){
  
    if (filtered_delta < WARM_THRESHOLD_3) {
      Serial.println("Hoooottt");
      digitalWrite(PIN_WARM1, HIGH);  // turn the LED off
      digitalWrite(PIN_WARM2, HIGH);  // turn the LED off
      digitalWrite(PIN_WARM3, HIGH);  // turn the LED off
      colder_led_off();
    } 
    else if (filtered_delta < WARM_THRESHOLD_2) {
      Serial.println("Hooot");
      digitalWrite(PIN_WARM1, HIGH);  // turn the LED off
      digitalWrite(PIN_WARM2, HIGH);  // turn the LED off
      digitalWrite(PIN_WARM3, LOW);  // turn the LED off
      colder_led_off();
    } 
    else if (filtered_delta < WARM_THRESHOLD_1) {
      Serial.println("Hot");
      digitalWrite(PIN_WARM1, HIGH);  // turn the LED off
      digitalWrite(PIN_WARM2, LOW);  // turn the LED off
      digitalWrite(PIN_WARM3, LOW);  // turn the LED off
      colder_led_off();
    } 
     else if (filtered_delta > COLD_THRESHOLD_3 ) {
      Serial.println("Collllldeeer");
      warmer_led_off();
      digitalWrite(PIN_COLD1, HIGH); // turn the LED on
      digitalWrite(PIN_COLD2, HIGH); // turn the LED on
      digitalWrite(PIN_COLD3, HIGH); // turn the LED on
    }
    else if (filtered_delta > COLD_THRESHOLD_2 ) {
      Serial.println("Colder");
      warmer_led_off();
      digitalWrite(PIN_COLD1, HIGH); // turn the LED on
      digitalWrite(PIN_COLD2, HIGH); // turn the LED on
      digitalWrite(PIN_COLD3, LOW); // turn the LED on
    }
    else if (filtered_delta > COLD_THRESHOLD_1 ) {
      Serial.println("Cold");
      warmer_led_off();
      digitalWrite(PIN_COLD1, HIGH); // turn the LED on
      digitalWrite(PIN_COLD2, LOW); // turn the LED on
      digitalWrite(PIN_COLD3, LOW); // turn the LED on
    }
    else {
      warmer_led_off();
      colder_led_off();
    }
  
}

void shift_values(){
  
    distance_n_minus_5 = distance_n_minus_4;
    distance_n_minus_4 = distance_n_minus_3;
    distance_n_minus_3 = distance_n_minus_2;
    distance_n_minus_2 = distance_n_minus_1;
    distance_n_minus_1 = new_distance;

    active_ping_n_minus_5 = active_ping_n_minus_4;
    active_ping_n_minus_4 = active_ping_n_minus_3;
    active_ping_n_minus_3 = active_ping_n_minus_2;
    active_ping_n_minus_2 = active_ping_n_minus_1;
    active_ping_n_minus_1 = new_active_ping;
  
}

double filtered_delta_distance(float d1, float d2, float d3, float d4, float d5, float d6){
   double filtered_delta=((0.2*(d1-d2))+(0.2*(d2-d3))+(0.2*(d3-d4))+(0.2*(d4-d5))+(0.2*(d5-d6)))*3.5;
   return filtered_delta;
}

double filtered_pings(float d1, float d2, float d3, float d4, float d5, float d6){
   double filtered_ping=((0.35*(d1))+(0.25*(d2))+(0.2*(d3))+(0.125*(d4))+(0.075*(d5)));
   return filtered_ping;
}

void colder_led_off(){
    digitalWrite(PIN_COLD1, LOW); // turn the LED on
    digitalWrite(PIN_COLD2, LOW); // turn the LED on
    digitalWrite(PIN_COLD3, LOW); // turn the LED on
}

void warmer_led_off(){
    digitalWrite(PIN_WARM1, LOW); // turn the LED on
    digitalWrite(PIN_WARM2, LOW); // turn the LED on
    digitalWrite(PIN_WARM3, LOW); // turn the LED on
}
