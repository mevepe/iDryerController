#include "servo.h"

Servo::Servo(uint8_t servoPin)
{
  _servoPin = servoPin;
  _moveDuration = 2000; // Длительность изменения положения сервопривода в миллисекундах
}

ServoState Servo::getState() const
{
  return _state;
}

void Servo::set(uint16_t closedDuration, uint16_t openedDuration, uint16_t openedAngle)
{
  _closedDuration = closedDuration;
  _openedDuration = openedDuration;
  _openedAngle = openedAngle;
}

void Servo::update()
{
  auto currentTimeMs = millis();

  if (_state == UNKNOWN)
  {
    close();
  }

  if (_state == MOVE)
  {
    _pulseWidth = map(_targetAngle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    PORTD |= (1 << _servoPin);
    Timer1.setPeriod(_pulseWidth); // Используем общий таймер для управления сервоприводом

    if (_nextMoveTimeMs < currentTimeMs)
    {
      _state = (_targetAngle == _openedAngle) ? OPEN : CLOSED;
    }
  }

  if (_state == CLOSED && _nextToggleTimeMs < currentTimeMs && _openedDuration > 0)
  {
    toggle();
    _nextToggleTimeMs = currentTimeMs + _openedDuration * 1000UL * 60UL;
  }

  if (_state == OPEN && _nextToggleTimeMs < currentTimeMs && _closedDuration > 0)
  {
    toggle();
    _nextToggleTimeMs = currentTimeMs + _closedDuration * 1000UL * 60UL;
  }

  if (currentTimeMs - _prevLogTimeMs >= 100)
  {
    _prevLogTimeMs = currentTimeMs;

#if KASYAK_FINDER && SERVO_LOGS
    Serial.print(" t: ");
    Serial.print(currentTimeMs);
    Serial.print(" s: ");
    Serial.print(_state);
    Serial.print(" a: ");
    Serial.print(_targetAngle);
    Serial.print(" p: ");
    Serial.print(_pulseWidth);
    Serial.print(" nt: ");
    Serial.print(_nextToggleTimeMs);
    Serial.print(" nm: ");
    Serial.print(_nextMoveTimeMs);
    Serial.println();
    Serial.flush();
#endif
  }
}

void Servo::toggle()
{
  if (_state == CLOSED)
  {
    open();
  }
  if (_state == OPEN)
  {
    close();
  }
}

void Servo::open()
{
  _targetAngle = _openedAngle;
  _state = MOVE;
  _nextMoveTimeMs = millis() + _moveDuration;
}

void Servo::close()
{
  _targetAngle = _closedAngle;
  _state = MOVE;
  _nextMoveTimeMs = millis() + _moveDuration;
}
