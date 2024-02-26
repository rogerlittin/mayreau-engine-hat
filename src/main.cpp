// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <esp_task_wdt.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>

#include "eh_analog.h"
#include "eh_digital.h"
#include "eh_display.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp/transforms/threshold.h"

using namespace sensesp;

// 5 second WDT
#define WDT_TIMEOUT 5

// 1-Wire data pin on SH-ESP32
#define ONEWIRE_PIN 4

// I2C pins on SH-ESP32
const int kSDAPin = 16;
const int kSCLPin = 17;

// ADS1115 I2C address
const int kADS1115Address = 0x4b;

// CAN bus (NMEA 2000) pins on SH-ESP32
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

// Engine hat digital input pins
const int kDigitalInputPin1 = GPIO_NUM_15;
const int kDigitalInputPin2 = GPIO_NUM_13;
const int kDigitalInputPin3 = GPIO_NUM_14;
const int kDigitalInputPin4 = GPIO_NUM_12;

const double fuel_capacity = 70.0;
const double black_capacity = 42.0;

// Test output pin configuration
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_18;
// repetition interval in ms; corresponds to 1000/(2*5)=100 Hz
const int kTestOutputInterval = 5;
#endif

TwoWire* i2c;
tNMEA2000* nmea2000;

reactesp::ReactESP app;

// Convenience function to print the addresses found on the I2C bus
void ScanI2C(TwoWire* i2c) {
  uint8_t error, address;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    i2c->beginTransmission(address);
    error = i2c->endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("done");
}

#ifdef ENABLE_TEST_OUTPUT_PIN
void ToggleTestOutputPin(void * parameter) {
  while (true) {
    digitalWrite(kTestOutputPin, !digitalRead(kTestOutputPin));
    delay(kTestOutputInterval);
  }
}
#endif

double oil_pressure = N2kDoubleNA;
double oil_temperature = N2kDoubleNA;
double coolant_temperature = N2kDoubleNA;
double exhaust_temperature = N2kDoubleNA;
double engine_rpm = N2kDoubleNA;
double fuel_level = N2kDoubleNA;
double black_level = N2kDoubleNA;
uint16_t status = 0;

void SendEngineDynamicParams() {
  tN2kMsg N2kMsg;
  SetN2kEngineDynamicParam(N2kMsg,
                           0,  // instance of a single engine is always 0
                           oil_pressure,  // oil pressure
                           oil_temperature, // oil temperature
                           coolant_temperature, // coolant temperature
                           N2kDoubleNA,  // alternator voltage
                           N2kDoubleNA,  // fuel rate
                           N2kDoubleNA,  // engine hours
                           N2kDoubleNA,  // engine coolant pressure
                           N2kDoubleNA,  // engine fuel pressure
                           N2kInt8NA,    // engine load
                           N2kInt8NA,    // engine torque
                           (tN2kEngineDiscreteStatus1)status,
                           (tN2kEngineDiscreteStatus2)0);
  nmea2000->SendMsg(N2kMsg);
}

void SendEngineParamsRapid() {
  tN2kMsg N2kMsg;
  SetN2kEngineParamRapid(N2kMsg, 0, engine_rpm);
  nmea2000->SendMsg(N2kMsg);
}

void SendExhaustTemp() {
  tN2kMsg N2kMsg;
  SetN2kTemperature(N2kMsg,
                    1,                            // SID
                    2,                            // TempInstance
                    N2kts_ExhaustGasTemperature,  // TempSource
                    exhaust_temperature           // actual temperature
  );
  nmea2000->SendMsg(N2kMsg);

}

void SendFuelLevels() {
  tN2kMsg N2kMsg;
  SetN2kFluidLevel(N2kMsg, 0, N2kft_Fuel, fuel_level, fuel_capacity);
  nmea2000->SendMsg(N2kMsg);
}

void SendBlackWaterLevels() {
  tN2kMsg N2kMsg;
  SetN2kFluidLevel(N2kMsg, 0, N2kft_BlackWater, black_level, black_capacity);
  nmea2000->SendMsg(N2kMsg);
}

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  debugI("Configuring WDT with timeout - %d secs", WDT_TIMEOUT);
  esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts.
  esp_task_wdt_add(NULL); // add current thread to WDT watch.

  app.onRepeat(2000, []() {
    debugI("Resetting WDT");
    esp_task_wdt_reset();
  });

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

  // define 1-Wire temperature sensors that update every 1000 ms
  // and have specific web UI configuration paths

  auto engine_oil_temperature =
      new OneWireTemperature(dts, 1000, "/Engine/OilTemp/oneWire");
  auto engine_exhaust_temperature =
      new OneWireTemperature(dts, 1000, "/Engine/WetExhaustTemp/oneWire");

  // define metadata for sensors
  auto engine_oil_temperature_metadata =
      new SKMetadata("K",                       // units
                     "Engine Oil Temperature",  // display name
                     "Engine Oil Temperature",  // description
                     "Oil Temperature",         // short name
                     10.                        // timeout, in seconds
      );

  auto engine_exhaust_temperature_metadata =
      new SKMetadata("K",                        // units
                     "Wet Exhaust Temperature",  // display name
                     "Wet Exhaust Temperature",  // description
                     "Exhaust Temperature",      // short name
                     10.                         // timeout, in seconds
      );

  // connect the sensors to Signal K output paths
  engine_oil_temperature->connect_to(new SKOutput<float>(
      "propulsion.engine.oilTemperature", "/Engine/OilTemp/skPath",
      engine_oil_temperature_metadata));

  // propulsion.*.wetExhaustTemperature is a non-standard path
  engine_exhaust_temperature->connect_to(
      new SKOutput<float>("propulsion.engine.wetExhaustTemperature",
                          "/Engine/WetExhaustTemp/skPath",
                          engine_exhaust_temperature_metadata));

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  ScanI2C(i2c);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(GAIN_ONE);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  xTaskCreate(ToggleTestOutputPin, "toggler", 2048, NULL, 1, NULL);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("engine-hat")
                    ->enable_ota("[My-Password]")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    // ->set_wifi("My WiFi SSID", "my_wifi_password")
                    // ->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  // Connect the analog senders
  auto engine_oil_pressure = ConnectPressureSender(ads1115, 0, "engine");
  auto engine_temperature = ConnectTempSender(ads1115, 1, "engine");
  auto fuel_tank_level = ConnectTankSender(ads1115, 2, "fuel");
  auto black_water_level = ConnectTankSender(ads1115, 3, "blackwater");

  // Connect the tacho senders
  auto tacho_frequency = ConnectTachoSender(kDigitalInputPin2, "engine");

  // Connect the alarm inputs
  // auto alarm_2_input = ConnectAlarmSender(kDigitalInputPin2, "2");
  auto engine_temperature_alarm = ConnectAlarmSender(kDigitalInputPin3, "engine.temperature");
  auto engine_oil_pressure_alarm = ConnectAlarmSender(kDigitalInputPin4, "engine.oilPressure");

  // Update the alarm states based on the input value changes
  // alarm_2_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));

  // initialize the NMEA 2000 subsystem

  // instantiate the NMEA2000 object
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  // Reserve enough buffer for sending all messages. This does not work on small
  // memory devices like Uno or Mega
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  nmea2000->SetProductInformation(
      "20230323",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SH-ESP32 Engine Interface",  // Manufacturer's Model ID (max 33 chars)
      "0.0.0.1 (2023-03-23)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.0.0.1 (2023-03-23)"   // Manufacturer's Model version (max 24 chars)
  );
  // Set device information
  nmea2000->SetDeviceInformation(
      1,    // Unique number. Use e.g. Serial number.
      130,  // Device function=Analog to NMEA 2000 Gateway. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      75,   // Device class=Inter/Intranetwork Device. See codes on
           // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      2046  // Just choosen free from code list on
            // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 22);
  // Disable all msg forwarding to USB (=Serial)
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() { 
    nmea2000->ParseMessages(); 
  });

  app.onRepeat(100, []() {
    SendEngineParamsRapid();
  });

  app.onRepeat(1000, []() {
    SendEngineDynamicParams();
    SendExhaustTemp();
    SendFuelLevels();
    SendBlackWaterLevels();
  });

  engine_oil_pressure->connect_to(
      new LambdaConsumer<float>([](float pressure) {
        oil_pressure = pressure;
      }));

  engine_oil_temperature->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        oil_temperature = temperature;
      }));

  engine_temperature->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        coolant_temperature = temperature;
      }));

  // hijack the exhaust gas temperature for wet exhaust temperature
  // measurement
  engine_exhaust_temperature->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        exhaust_temperature = temperature;
      }));

  auto exhaust_temp_threshold = new FloatThreshold(273.15, 323.15, false, "/Engine/WetExhaustTemp/Threshold");

  engine_exhaust_temperature->connect_to(exhaust_temp_threshold)->connect_to(
      new LambdaConsumer<bool>([](bool state) {
        if (state) // trigger coolant flow alarm
          status |= 1 << 7;
        else
          status &= ~(1 << 7);
      }));

  fuel_tank_level->connect_to(
      new LambdaConsumer<float>([](float level) {
        fuel_level = level;
      }));

  black_water_level->connect_to(
      new LambdaConsumer<float>([](float level) {
        black_level = level;
      }));

  tacho_frequency->connect_to(
      new LambdaConsumer<float>([](float value) {
        engine_rpm = 60 * value; 
      }));

  engine_temperature_alarm->connect_to(
    new LambdaConsumer<bool>([](bool state) {
      if (state)
        status |= 1 << 1;
      else
        status &= ~(1 << 1);
    }));

  engine_oil_pressure_alarm->connect_to(
    new LambdaConsumer<bool>([](bool state) {
      if (state && engine_rpm > 50.0)
        status |= 1 << 2;
      else
        status &= ~(1 << 2);
    }));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
