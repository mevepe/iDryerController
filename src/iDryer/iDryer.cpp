#include "iDryer.h"

iDryer::iDryer(thermistor &heaterTempSensor, AirTempSensor &airTempSensor) : _heaterTempSensor(heaterTempSensor), _airTempSensor(airTempSensor)
{
}

float iDryer::GetSetpoint() const
{
  return _targetHeaterTemp;
}
float iDryer::GetOutput() const
{
  return _heaterOutput;
}

void iDryer::SetOutput(float output)
{
  _heaterOutput = output;
  _dimmer = uint16_t(heaterPid.GetMappedOutput(HEATER_MAX, HEATER_MIN));
}

uint16_t iDryer::getPulseWidth() const
{
  return _dimmer;
}

bool iDryer::IsHeatingAllowed() const
{
  return (state == DRY || state == STORAGE || state == AUTOPID) && _dimmer >= HEATER_MIN && _dimmer < HEATER_OFF;
}

bool iDryer::UpdateData()
{
  data.timestamp = millis();
  data.ntcTemp = (_heaterTempSensor.analog2temp() + data.ntcTemp) / 2.0f;

#ifdef SENSOR_SHT31
  if (_airTempSensor.dataReady())
  {
    _airTempSensor.read();
    data.airTemp = (_airTempSensor.getTemperature() + data.airTemp) / 2.0f;
    data.airHumidity = (_airTempSensor.getHumidity() + data.airHumidity) / 2.0f;
  }
#endif

#ifdef SENSOR_BME280
  data.airTemp = (_airTempSensor.readTemperature() + data.airTemp) / 2.0f;
  data.airHumidity = (_airTempSensor.readHumidity() + data.airHumidity) / 2.0f;
#endif

  data.airTempCorrected = data.airTemp;
  data.flagScreenUpdate = false;

  if (data.airTemp > MIN_CALIB_TEMP)
  {
    data.airTempCorrected = math::map_to_range_with_clamp(data.airTemp, MIN_CALIB_TEMP, MAX_CALIB_TEMP, REAL_CALIB_TEMP_MIN, REAL_CALIB_TEMP_MAX);
  }

  if (data.timestamp - _lastScreenUpdateTimestamp > SCREEN_UPADATE_TIME)
  {
    _lastScreenUpdateTimestamp = data.timestamp;
    data.flagScreenUpdate = true;
  }

  if (!data.flagTimeCounter && (uint8_t(round(data.airTempCorrected)) >= data.setTemp - DRY_START_THRESHOLD))
  {
    data.flagTimeCounter = true;
  }

  if (data.ntcTemp < TMP_MIN)
  {
    return false;
  }

  if (data.ntcTemp > TMP_MAX + TMP_SAFETY_THRESHOLD)
  {
    return false;
  }

  if (data.airTempCorrected < TMP_MIN)
  {
    return false;
  }

  if (data.airTempCorrected > TMP_MAX + TMP_SAFETY_THRESHOLD)
  {
    return false;
  }

  return true;
}

void iDryer::Setpoint()
{
  _time = data.timestamp / float(math::msCountInSec); // Текущее время
  _currentAirTemp = data.airTempCorrected;            // Текущая температура
  _targetAirTemp = data.setTemp;                      // Заданная температура
  _currentHeaterTemp = data.ntcTemp;                  // Текущая температура нагревателя

  auto airTempError = _targetAirTemp - _currentAirTemp;
  airPid.Process(_time, airTempError);

  if (airPid.IsOutputUpdated())
  {
    _targetHeaterTemp = airPid.GetMappedOutput(0.0f, _targetAirTemp + data.deltaT);
  }

  // Отключение при критическом перегреве
  if (_currentAirTemp >= _targetAirTemp + CRITICAL_OVERHEAT)
  {
    _targetHeaterTemp = 0;
  }

  auto heaterTempError = _targetHeaterTemp - _currentHeaterTemp;
  heaterPid.Process(_time, heaterTempError);

  if (heaterPid.IsOutputUpdated())
  {
    SetOutput(heaterPid.GetOutput());

    if (_targetHeaterTemp == 0)
    {
      _dimmer = HEATER_OFF;
    }

#if KASYAK_FINDER && DRY_AIR_LOGS
    Serial.print(" t: ");
    Serial.print(data.timestamp);
    Serial.print(" d: ");
    Serial.print(airTempError, 2);
    Serial.print(" t: ");
    Serial.print(_currentAirTemp, 2);
    Serial.print(" s: ");
    Serial.print(_targetHeaterTemp, 2);
    Serial.print(" n: ");
    Serial.print(_currentHeaterTemp, 2);
    Serial.print(" dt: ");
    Serial.print(airPid.GetDeltaTime(), 3);
    Serial.print(" pt: ");
    Serial.print(airPid.GetProportionalTerm(), 3);
    Serial.print(" it: ");
    Serial.print(airPid.GetIntegralTerm(), 3);
    Serial.print(" dt: ");
    Serial.print(airPid.GetDerivativeTerm(), 3);
    Serial.print(" ft: ");
    Serial.print(airPid.GetFilterTerm(), 2);
    Serial.print(" o: ");
    Serial.print(airPid.GetOutput(), 2);
    Serial.println();
    Serial.flush();
#endif

#if KASYAK_FINDER && DRY_HEATER_LOGS
    Serial.print(" t: ");
    Serial.print(data.timestamp);
    Serial.print(" d: ");
    Serial.print(airTempError, 2);
    Serial.print(" t: ");
    Serial.print(_currentAirTemp, 2);
    Serial.print(" s: ");
    Serial.print(_targetHeaterTemp, 2);
    Serial.print(" n: ");
    Serial.print(_currentHeaterTemp, 2);
    Serial.print(" dt: ");
    Serial.print(heaterPid.GetDeltaTime(), 3);
    Serial.print(" pt: ");
    Serial.print(heaterPid.GetProportionalTerm(), 3);
    Serial.print(" it: ");
    Serial.print(heaterPid.GetIntegralTerm(), 3);
    Serial.print(" dt: ");
    Serial.print(heaterPid.GetDerivativeTerm(), 3);
    Serial.print(" ft: ");
    Serial.print(heaterPid.GetFilterTerm(), 2);
    Serial.print(" o: ");
    Serial.print(heaterPid.GetOutput(), 2);
    Serial.print(" d: ");
    Serial.print(_dimmer);
    Serial.println();
    Serial.flush();
#endif
  }
}
