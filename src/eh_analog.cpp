#include "eh_analog.h"

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"


// ADS1115 input hardware scale factor (input voltage vs voltage at ADS1115)
const float kAnalogInputScale = 29. / 2.048;

// Engine Hat constant measurement current (A)
const float kMeasurementCurrent = 0.01;

// Default fuel tank size, in m3
const float kTankDefaultSize = 70. / 1000;

FloatProducer* ConnectTankSender(Adafruit_ADS1115* ads1115, int channel, String name) {
  const uint ads_read_delay = 500;  // ms

  char config_path[80];
  char sk_path[80];
  char meta_display_name[80];
  char meta_description[80];

  auto sender_resistance =
      new RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
      });

  snprintf (config_path, sizeof(config_path), "/Tank %s/Sender Scale", name.c_str());
  auto sender_scale = new Linear(1, 0, config_path);

  snprintf(config_path, sizeof(config_path), "/Tank %s/Resistance SK Path",name.c_str());
  snprintf(sk_path, sizeof(sk_path), "tanks.%s.senderResistance",name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "Resistance %s",name.c_str());
  snprintf(meta_description, sizeof(meta_description),"Measured tank %s sender resistance", name.c_str());
  auto sender_resistance_sk_output = new SKOutputFloat(sk_path, config_path,
      new SKMetadata("ohm", meta_display_name, meta_description));

  snprintf(config_path, sizeof(config_path), "/Tank %s/Level Curve", name.c_str());
  auto tank_level = (new CurveInterpolator(nullptr, config_path))
                        ->set_input_title("Sender Resistance (ohms)")
                        ->set_output_title("Fuel Level (ratio)");

  if (tank_level->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    tank_level->clear_samples();
    tank_level->add_sample(CurveInterpolator::Sample(10, 0));
    tank_level->add_sample(CurveInterpolator::Sample(180., 1));
  }

  snprintf(config_path, sizeof(config_path), "/Tank %s/Current Level SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "tanks.%s.currentLevel", name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "Tank %s level", name.c_str());
  snprintf(meta_description, sizeof(meta_description), "Tank %s level", name.c_str());
  auto tank_level_sk_output = new SKOutputFloat(sk_path, config_path,
      new SKMetadata("ratio", meta_display_name, meta_description));

  snprintf(config_path, sizeof(config_path), "/Tank %s/Total Volume", name.c_str());
  auto tank_volume = new Linear(kTankDefaultSize, 0, config_path);

  snprintf(config_path, sizeof(config_path), "/Tank %s/Current Volume SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "tanks.%s.currentVolume", name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "Tank %s volume", name.c_str());
  snprintf(meta_description, sizeof(meta_description), "Calculated tank %s remaining volume", name.c_str());
  auto tank_volume_sk_output = new SKOutputFloat(sk_path, config_path,
      new SKMetadata("m3", meta_display_name, meta_description));

  sender_resistance->connect_to(sender_scale)->connect_to(sender_resistance_sk_output);

  sender_resistance->connect_to(sender_scale)->connect_to(tank_level)->connect_to(tank_level_sk_output);

  tank_level->connect_to(tank_volume)->connect_to(tank_volume_sk_output);

  return tank_level;
}

FloatProducer* ConnectTempSender(Adafruit_ADS1115* ads1115, int channel, String name) {
  const uint ads_read_delay = 500;  // ms

  char config_path[80];
  char sk_path[80];
  char meta_display_name[80];
  char meta_description[80];

  snprintf(config_path, sizeof(config_path), "/Temp %s/Resistance", name.c_str());
  auto sender_resistance =
      new RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
      });

  snprintf(config_path, sizeof(config_path), "/Temp %s/Resistance SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.temperature.senderResistance", name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "%s Temp Resistance", name.c_str());
  snprintf(meta_description, sizeof(meta_description), "%s Measured temp sender resistance", name.c_str());
  
  auto sender_resistance_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("ohm", meta_display_name, meta_description));

  snprintf(config_path, sizeof(config_path), "/Temp %s/Level Curve", name.c_str());
  auto temperature = (new CurveInterpolator(nullptr, config_path))
                        ->set_input_title("Sender Resistance (ohms)")
                        ->set_output_title("Temperature (Kelvin)");

  if (temperature->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    // + 273.15 om van C naar F voor signalk te komen.
    temperature->clear_samples();
    temperature->add_sample(CurveInterpolator::Sample(281, 273.15 + 40));
    temperature->add_sample(CurveInterpolator::Sample(68, 273.15 + 80));
    temperature->add_sample(CurveInterpolator::Sample(22, 273.15 + 120));
  }

  snprintf(config_path, sizeof(config_path), "/Temp %s/Current Value SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.temperature", name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "%s temperature", name.c_str());
  snprintf(meta_description, sizeof(meta_description), "%s temperature", name.c_str());
  auto temperature_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("K", meta_display_name, meta_description));

  sender_resistance->connect_to(sender_resistance_sk_output);
  sender_resistance->connect_to(temperature)->connect_to(temperature_sk_output);

  return temperature;
}

FloatProducer* ConnectPressureSender(Adafruit_ADS1115* ads1115, int channel, String name) {
  const uint ads_read_delay = 500;  // ms

  char config_path[80];
  char sk_path[80];
  char meta_display_name[80];
  char meta_description[80];

  snprintf(config_path, sizeof(config_path), "/Oil Pressure %s/Resistance", name.c_str());
  auto sender_resistance =
      new RepeatSensor<float>(ads_read_delay, [ads1115, channel]() {
        int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        float adc_output_volts = ads1115->computeVolts(adc_output);
        return kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
      });

  snprintf(config_path, sizeof(config_path), "/Oil Pressure %s/Resistance SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.oilPressure.senderResistance", name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "%s Oil Pressure Resistance", name.c_str());
  snprintf(meta_description, sizeof(meta_description), "Measured %s oil pressure sender resistance", name.c_str());
  
  auto sender_resistance_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("ohm", meta_display_name, meta_description));

  snprintf(config_path, sizeof(config_path), "/Oil Pressure %s/Curve", name.c_str());
  auto pressure = (new CurveInterpolator(nullptr, config_path))
                        ->set_input_title("Sender Resistance (ohms)")
                        ->set_output_title("Pressure (Pascals)");

  if (pressure->get_samples().empty()) {
    // If there's no prior configuration, provide a default curve
    pressure->clear_samples();
    pressure->add_sample(CurveInterpolator::Sample(10, 0));
    pressure->add_sample(CurveInterpolator::Sample(90, 50000));
    pressure->add_sample(CurveInterpolator::Sample(180, 1000000));
}

  snprintf(config_path, sizeof(config_path), "/Oil Pressure %s/Current SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.oilPressure", name.c_str());
  snprintf(meta_display_name, sizeof(meta_display_name), "%s Oil Pressure", name.c_str());
  snprintf(meta_description, sizeof(meta_description), "%s Oil Pressure", name.c_str());
  auto pressure_sk_output = new SKOutputFloat(
      sk_path, config_path,
      new SKMetadata("Pa", meta_display_name, meta_description));

  sender_resistance->connect_to(sender_resistance_sk_output);
  sender_resistance->connect_to(pressure)->connect_to(pressure_sk_output);

  return pressure;
}
