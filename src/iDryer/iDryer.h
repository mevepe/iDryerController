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

#ifdef SENSOR_BME280
#define AirTempSensor GyverBME280
#else
#define AirTempSensor SHT31
#endif

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
  PIDController airPid;
  PIDController heaterPid;

private:
  unsigned long _lastScreenUpdateTimestamp = 0;

  float _time = 0;
  float _targetAirTemp = 0;
  float _currentAirTemp = 0;
  float _targetHeaterTemp = 0;
  float _currentHeaterTemp = 0;
  float _heaterOutput = 0;

  uint16_t _dimmer = 0;

  thermistor &_heaterTempSensor;
  AirTempSensor &_airTempSensor;

public:
  iDryer(thermistor &heaterTempSensor, AirTempSensor &airTempSensor);

  float GetSetpoint() const;

  float GetOutput() const;
  void SetOutput(float output);

  uint16_t getPulseWidth() const;

  bool IsHeatingAllowed() const;

  bool getData();

  void Setpoint();
};

#endif // IDRYER_H
