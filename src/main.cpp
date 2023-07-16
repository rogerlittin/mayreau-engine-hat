// SenESP Engine Sensors

#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>  //JG Added
#include <Arduino.h>


#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/transforms/frequency.h"


using namespace sensesp;

reactesp::ReactESP app;

// BME280
  
    Adafruit_BME280 bme280;

  float read_temp_callback() { return (bme280.readTemperature() + 273.15);}
  float read_pressure_callback() { return (bme280.readPressure());}
  float read_humidity_callback() { return (bme280.readHumidity());}


// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_ENABLED
  SetupSerialDebug(115200);
#endif

  Wire.begin(21,22);                // join i2c bus (address optional for master)
// 
 //Serial.begin(9600);          // start serial communication at 9600bps
  
  //Serial.println(F("BME280 Forced Mode Test."));

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  //if (!bme280.begin()) {
    //Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
      //                "try a different address!"));
    //while (1) delay(10);// could need a delay here:
  //}   
//
 
  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("SensESP")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    // ->set_sk_server("raspberrypi4.local", 3000)
                    ->enable_uptime_sensor()
                    ->get_app();

/// BME280 SENSOR CODE - Temp/Humidity/Altitude/Pressure Sensor ////  

  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  // If you need to use the TwoWire library instead of the Wire library, there
  // is a different constructor: see bmp280.h

  bme280.begin();
  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* bme280_temp =
      new RepeatSensor<float>(5000, read_temp_callback);

  auto* bme280_pressure = 
      new RepeatSensor<float>(60000, read_pressure_callback);

  auto* bme280_humidity = 
      new RepeatSensor<float>(60000, read_humidity_callback);     

  // Send the temperature to the Signal K server as a Float
  bme280_temp->connect_to(new SKOutputFloat("environment.inside.temperature"));

  bme280_pressure->connect_to(new SKOutputFloat("environment.inside.pressure"));

  bme280_humidity->connect_to(new SKOutputFloat("environment.inside.relativeHumidity"));

//// Pressure Sender Config ////

// const float Vin = 3.28;
// const float R1 = 47.0;
// auto* analog_input = new AnalogInput(36, 500); //- Pin 36 is Analogue 0

// analog_input->connect_to(new AnalogVoltage(Vin,Vin))
//       ->connect_to(new VoltageDividerR2(R1, Vin, "/Water Tank/sender"))
//       ->connect_to(new LevelInterpreter("/Water Tank/curve"))
//       ->connect_to(new Linear(1.0, 0.0, "/Water Tank/calibrate"))
//       ->connect_to(new SKOutputFloat("tanks.freshwater.currentLevel", "/Water Tank/sk_path"));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();

}

void loop() { app.tick(); }