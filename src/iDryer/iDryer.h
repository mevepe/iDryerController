#ifndef IDRYER_H
#define IDRYER_H

#include <Arduino.h>
#include <GyverTimers.h>
#include "Configuration.h"
#include "math/math_extensions.h"
#include "math/algorithms/pid/pid.h"
#include "thermistor/thermistor.h"
#include "GyverBME280.h"
#include "SHT31.h"

using math::algorithms::PIDController;

enum State
{
  OFF,
  ON,
  MENU,
  DRY,
  STORAGE,
  AUTOPID,
  NTC_ERROR,
};

struct Data
{
  unsigned long timestamp = 0; // timestamp in ms
  float ntcTemp = 0;
  float airTemp = 0;
  float airTempCorrected = 0;
  float airHumidity = 0;
  bool optimalConditionsReachedFlag = false;
  unsigned long startTime = 0;
  uint8_t setTemp = 0;
  uint8_t setHumidity = 0;
  uint16_t setTime = 0;
  bool flag = false;
  bool flagScreenUpdate = false;
  bool flagTimeCounter = false;
  uint8_t setFan = 0;
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float Kf = 0;
  float minDeltaTime = 0;
  uint8_t deltaT = 0;
};

class iDryer
{
  // TODO: Prefer getters and setters for data members instead of direct access
public:
  State state = OFF;
  Data data;
  PIDController pid;

private:
  thermistor &_ntc;

#ifdef SENSOR_BME280
  GyverBME280 &_bme;
#else
  SHT31 &sht;
#endif

  unsigned long _lastScreenUpdateTimestamp = 0;

  float _setpoint = 0;
  float _input = 0;
  float _output = 0;

  uint16_t _dimmer = 0;

public:
#ifdef SENSOR_BME280
  iDryer(thermistor &ntc, GyverBME280 &bme);
#else
  iDryer(thermistor &ntc, SHT31 &sht);
#endif

  float GetSetpoint() const;

  float GetOutput() const;
  void SetOutput(float output);

  uint16_t getPulseWidth() const;

  bool IsHeatingAllowed() const;

  bool getData();

  void Setpoint();
};

#endif // IDRYER_H
