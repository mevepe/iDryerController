#include "iDryer.h"

iDryer::iDryer(thermistor &heaterTempSensor, AirTempSensor &airTempSensor) : _heaterTempSensor(heaterTempSensor), _airTempSensor(airTempSensor)
{
  auto outputRange = airPid.GetMaxOutput() - airPid.GetMinOutput();

  airPid.SetProportionalGain(0.0f); // используем I регулятор для температуры воздуха
  airPid.SetIntegralGain(0.01f);    // небольшая интегральная составляющая для устранения статической ошибки
  airPid.SetDerivativeGain(0.0f);   // используем I регулятор для температуры воздуха
}

float iDryer::GetSetpoint() const
{
  return _targetHeaterTemp;
}

float iDryer::GetOutput() const
{
  return _output;
}

void iDryer::SetOutput(float output)
{
  _output = output;
  _overrideOutput = true;
}

bool iDryer::IsHeatingAllowed() const
{
  return state == DRY || state == STORAGE || state == AUTOPID;
}

void iDryer::Reset()
{
  _lastScreenUpdateTimestamp = 0;
  _targetHeaterTemp = 0;
  _output = 0;
  _overrideOutput = false;

  airPid.Reset();
  heaterPid.Reset();
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
  auto time = data.timestamp / float(math::msCountInSec); // Текущее время
  auto currentAirTemp = data.airTempCorrected;            // Текущая температура
  auto targetAirTemp = data.setTemp;                      // Заданная температура
  auto currentHeaterTemp = data.ntcTemp;                  // Текущая температура нагревателя

  auto airTempError = targetAirTemp - currentAirTemp;
  airPid.Process(time, airTempError);

  if (airPid.IsOutputUpdated())
  {
    _targetHeaterTemp = targetAirTemp;                                            // Целевая температура нагревателя
    _targetHeaterTemp += airPid.GetMappedOutput(-HEATING_THRESHOLD, data.deltaT); // Компенсация статической ошибки
  }

  if (_targetHeaterTemp > TMP_MAX)
  {
    _targetHeaterTemp = TMP_MAX;
  }

  // Отключение при критическом перегреве
  if (currentAirTemp >= targetAirTemp + CRITICAL_OVERHEAT)
  {
    _targetHeaterTemp = 0;
  }

  auto heaterTempError = _targetHeaterTemp - currentHeaterTemp;
  heaterPid.Process(time, heaterTempError);

  if (heaterPid.IsOutputUpdated())
  {
    if (!_overrideOutput)
    {
      _output = heaterPid.GetOutput();
    }

    if (_targetHeaterTemp == 0)
    {
      _output = 0;
    }

#if KASYAK_FINDER && DRY_AIR_LOGS
    Serial.print(" t: ");
    Serial.print(data.timestamp);
    Serial.print(" d: ");
    Serial.print(airTempError, 2);
    Serial.print(" at: ");
    Serial.print(currentAirTemp, 2);
    Serial.print(" s: ");
    Serial.print(_targetHeaterTemp, 2);
    Serial.print(" n: ");
    Serial.print(currentHeaterTemp, 2);
    Serial.print(" dt: ");
    Serial.print(airPid.GetDeltaTime(), 3);
    Serial.print(" pp: ");
    Serial.print(airPid.GetProportionalTerm(), 3);
    Serial.print(" pi: ");
    Serial.print(airPid.GetIntegralTerm(), 3);
    Serial.print(" pd: ");
    Serial.print(airPid.GetDerivativeTerm(), 3);
    Serial.print(" pf: ");
    Serial.print(airPid.GetFilterTerm(), 2);
    Serial.print(" po: ");
    Serial.print(airPid.GetOutput(), 2);
    Serial.println();
    Serial.flush();
#endif

#if KASYAK_FINDER && DRY_HEATER_LOGS
    if (Serial.available())
    {
      auto input = Serial.readStringUntil('\n');

      auto ptr = input.c_str();
      char *endPtr = nullptr;

      auto overrideOutput = static_cast<bool>(strtoul(ptr, &endPtr, 10));
      auto overrideOutputParsed = ptr != endPtr;
      ptr = endPtr;

      auto output = static_cast<float>(strtod(ptr, &endPtr));
      auto outputParsed = ptr != endPtr;
      ptr = endPtr;

      auto kp = static_cast<float>(strtod(ptr, &endPtr));
      auto kpParsed = ptr != endPtr;
      ptr = endPtr;

      auto ki = static_cast<float>(strtod(ptr, &endPtr));
      auto kiParsed = ptr != endPtr;
      ptr = endPtr;

      auto kd = static_cast<float>(strtod(ptr, &endPtr));
      auto kdParsed = ptr != endPtr;

      if (overrideOutputParsed && outputParsed && kpParsed && kiParsed && kdParsed)
      {
        _overrideOutput = overrideOutput;
        _output = output;
        heaterPid.SetProportionalGain(kp);
        heaterPid.SetIntegralGain(ki);
        heaterPid.SetDerivativeGain(kd);
      }
    }

    auto output = GetOutput();
    auto minOutput = heaterPid.GetMinOutput();
    auto maxOutput = heaterPid.GetMaxOutput();
    auto dutyCycle = math::map_to_range_with_clamp(output, minOutput, maxOutput, 0, HEATER_PERIOD_COUNT);

    auto zero_impulse_on_count = static_cast<uint16_t>(round(dutyCycle));

    Serial.print(" t: ");
    Serial.print(data.timestamp);
    Serial.print(" d: ");
    Serial.print(heaterTempError, 2);
    Serial.print(" at: ");
    Serial.print(currentAirTemp, 2);
    Serial.print(" s: ");
    Serial.print(_targetHeaterTemp, 2);
    Serial.print(" n: ");
    Serial.print(currentHeaterTemp, 2);
    Serial.print(" dt: ");
    Serial.print(heaterPid.GetDeltaTime(), 3);
    Serial.print(" pp: ");
    Serial.print(heaterPid.GetProportionalTerm(), 3);
    Serial.print(" pi: ");
    Serial.print(heaterPid.GetIntegralTerm(), 3);
    Serial.print(" pd: ");
    Serial.print(heaterPid.GetDerivativeTerm(), 3);
    Serial.print(" po: ");
    Serial.print(_output, 2);
    Serial.print(" ic: ");
    Serial.print(zero_impulse_on_count);
    Serial.println();
    Serial.flush();
#endif
  }
}
