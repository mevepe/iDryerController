#include "iDryer.h"

#ifdef SENSOR_BME280
iDryer::iDryer(thermistor &ntc, GyverBME280 &bme) : ntc(ntc), bme(bme)
{
}
#else
iDryer::iDryer(thermistor &ntc, SHT31 &sht) : ntc(ntc), sht(sht)
{
}
#endif

float iDryer::GetSetpoint() const
{
  return setpoint;
}
float iDryer::GetOutput() const
{
  return output;
}

void iDryer::SetOutput(float output)
{
  this->output = output;
  this->dimmer = uint16_t(math::map_to_range_with_clamp(output, pid.GetMinOutput(), pid.GetMaxOutput(), HEATER_MIN, HEATER_MAX));
}

uint16_t iDryer::getPulseWidth() const
{
  return dimmer;
}

bool iDryer::IsHeatingAllowed() const
{
  return (state == DRY || state == STORAGE || state == AUTOPID) && dimmer >= HEATER_MIN && dimmer < HEATER_MAX;
}

bool iDryer::getData()
{
  data.timestamp = millis();
  data.ntcTemp = (ntc.analog2temp() + data.ntcTemp) / 2.0f;

#ifdef SENSOR_SHT31
  if (sht.dataReady())
  {
    sht.read();
    data.airTemp = (sht.getTemperature() + data.airTemp) / 2.0f;
    data.airHumidity = (sht.getHumidity() + data.airHumidity) / 2.0f;
  }
#endif

#ifdef SENSOR_BME280
  data.airTemp = (bme.readTemperature() + data.airTemp) / 2.0f;
  data.airHumidity = (bme.readHumidity() + data.airHumidity) / 2.0f;
#endif

  data.airTempCorrected = data.airTemp;
  data.flagScreenUpdate = false;

  if (data.airTemp > MIN_CALIB_TEMP)
  {
    data.airTempCorrected = math::map_to_range_with_clamp(data.airTemp, MIN_CALIB_TEMP, MAX_CALIB_TEMP, REAL_CALIB_TEMP_MIN, REAL_CALIB_TEMP_MAX);
  }

  if (data.timestamp - lastScreenUpdateTimestamp > SCREEN_UPADATE_TIME)
  {
    lastScreenUpdateTimestamp = data.timestamp;
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
  auto currentTemp = data.airTempCorrected; // Текущая температура
  float desiredTemp = data.setTemp;         // Заданная температура
  float deltaT = data.deltaT;               // Дополнительный коэффициент для агрессивного нагрева

  auto delta = desiredTemp - currentTemp;
  auto adjustment = math::map_to_range_with_clamp(delta, 0.0f, HEATING_THRESHOLD, HEATER_AIR_DELTA, deltaT);

  if (delta < 0.0f)
  {
    adjustment -= math::map_to_range_with_clamp(abs(delta), 0.0f, 1.0f, 0.0f, HEATING_THRESHOLD);
  }

  setpoint = desiredTemp + adjustment;

  if (setpoint > TMP_MAX)
  {
    setpoint = TMP_MAX;
  }

  // Отключение при критическом перегреве
  if (currentTemp >= desiredTemp + CRITICAL_OVERHEAT)
  {
    setpoint = 0;
  }

  input = data.ntcTemp;

  auto timeInSeconds = data.timestamp / float(math::msCountInSec);
  auto heaterTempError = setpoint - input;
  pid.Process(timeInSeconds, heaterTempError);

  if (pid.IsOutputUpdated())
  {
    SetOutput(pid.GetOutput());

    if (setpoint == 0)
    {
      dimmer = HEATER_OFF;
    }

#if KASYAK_FINDER && DRY_LOGS
    Serial.print(" t: ");
    Serial.print(data.timestamp);
    Serial.print(" d: ");
    Serial.print(delta, 2);
    Serial.print(" a: ");
    Serial.print(adjustment, 2);
    Serial.print(" t: ");
    Serial.print(currentTemp, 2);
    Serial.print(" s: ");
    Serial.print(setpoint, 2);
    Serial.print(" n: ");
    Serial.print(input, 2);
    Serial.print(" dt: ");
    Serial.print(pid.GetDeltaTime(), 3);
    Serial.print(" pt: ");
    Serial.print(pid.GetProportionalTerm(), 3);
    Serial.print(" it: ");
    Serial.print(pid.GetIntegralTerm(), 3);
    Serial.print(" dt: ");
    Serial.print(pid.GetDerivativeTerm(), 3);
    Serial.print(" ft: ");
    Serial.print(pid.GetFilterTerm(), 2);
    Serial.print(" o: ");
    Serial.print(pid.GetOutput(), 2);
    Serial.print(" d: ");
    Serial.print(dimmer);
    Serial.println();
    Serial.flush();
#endif
  }
}
