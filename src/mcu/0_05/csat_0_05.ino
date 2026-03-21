// Compile for the Pi Pico 2040

#define SW_VER "Cubesat payload MCU 0.05"
#define SW_REL "4 Mar 2026"

#define DEBUG 1

#include <Wire.h>

///////////////////////////////////////////////////////////////////////////////
// Pi Zero UART
#define PiUART Serial1

///////////////////////////////////////////////////////////////////////////////
//  GPS

#include <Adafruit_GPS.h>
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

/////////////////////////////////////////////////////////////////////////////////
// IMU
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

/////////////////////////////////////////////////////////////////////
//  Voltage monitor
//https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/arduino-code
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  



////////////////////////////////////////////////////////////////////////////////
// Setup

#define LOG_INTERVAL 2000
uint32_t timer = millis();

void setup() {
  Serial.begin(115200); 
  Serial.println(SW_VER);
  Serial.println(SW_REL);

  Wire1.setSDA(14);
  Wire1.setSCL(15);
  Wire1.begin(); // Initializes I2C1

  // Send to Rpi on Serial2
  PiUART.begin(115200);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  if (!icm.begin_I2C(0x69, &Wire1)) {
    #ifdef DEBUG
    Serial.println("Failed to find ICM20948 chip");
    #endif
    for(;;);
    }

  if (!ads.begin(0x48, &Wire1)) {
    #ifdef DEBUG
    Serial.println("Failed to initialize ADS.");
    #endif
    for(;;);
    } 
  }

////////////////////////////////////////////////////////////////
// Loop

void loop() { 
  char buf[128];
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;

  int16_t adc0, adc1, adc2, adc3;
  int volts0, volts1, volts2, volts3;

  char c = GPS.read();
  
  if (GPSECHO) 
    if (c) {
      #ifdef DEBUG
      Serial.print(c);
      #endif
      }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    #ifdef DEBUG
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    #endif
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
    }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > LOG_INTERVAL) {
    timer = millis(); // reset the timer
    // Sample GPS output:
    //    !t:3/3/26,19:53:42.0
    //    !g:1,1,4522.486816,N,7545.881348,W,0.338000,77.199997,9

    sprintf(buf, "!t:%d/%d/%d,%02d:%02d:%02d.%d", // time UTC
                                GPS.day,
                                GPS.month,
                                GPS.year,
                                GPS.hour,
                                GPS.minute,
                                GPS.seconds,
                                GPS.milliseconds
                                );
    PiUART.println(buf);                        
    
    sprintf(buf, "!g:%d,%d,%f,%c,%f,%c,%f,%f,%d", 
                                (int) GPS.fix,   // boolean
                                GPS.fixquality,  //  (0, 1, 2 = Invalid, GPS, DGPS)
                                GPS.latitude,    // 45°22'28.8"N 75°45'52.9"W is formatted as 
                                GPS.lat,         // 4522.480,N,7545.882,W  (28.8/60=0.48, etc)
                                GPS.longitude,
                                GPS.lon,
                                GPS.speed,
                                GPS.altitude,
                                GPS.satellites
                                );
    PiUART.println(buf);
    #ifdef DEBUG
    Serial.println(buf);
    #endif

    icm.getEvent(&accel, &gyro, &temp, &mag);
    sprintf(buf, "!i:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", 
                                (int) (temp.temperature*100),
                                (int) (accel.acceleration.x*100),
                                (int) (accel.acceleration.y*100),
                                (int) (accel.acceleration.z*100),
                                (int) (mag.magnetic.x*100),
                                (int) (mag.magnetic.y*100),
                                (int) (mag.magnetic.z*100),
                                (int) (gyro.gyro.x*100),
                                (int) (gyro.gyro.y*100),
                                (int) (gyro.gyro.z*100)
                                );
    PiUART.println(buf);
    #ifdef DEBUG
    Serial.println(buf);
    #endif

    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);

    volts0 = (int) (ads.computeVolts(adc0)*1000);
    volts1 = (int) (ads.computeVolts(adc1)*1000);
    volts2 = (int) (ads.computeVolts(adc2)*1000);
    volts3 = (int) (ads.computeVolts(adc3)*1000);

    sprintf(buf, "!v:%d,%d,%d,%d", volts0, volts1, volts2, volts3);

    PiUART.println(buf);
    #ifdef DEBUG
    Serial.println(buf);
    #endif
    } 
  }