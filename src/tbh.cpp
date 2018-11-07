#include <Arduino.h>

#include "tbh.h"

TBH::TBH(double setPoint, double kp, double ki, unsigned long interval):
    _setPoint(setPoint), _kp(kp), _ki(ki), _interval(interval) {}

bool TBH::handle(double input, unsigned long t)
{
    if (!_autoMode || t < _nextUpdate)
        return false;
    double err = _setPoint - input;
    double fScaled = _ki * double(t - _nextUpdate + _interval) / _interval;
    _nextUpdate = t + _interval;
    _intTerm += fScaled * err;
    Serial.print("TBH: preliminary int term = ");
    Serial.println(_intTerm);
    if (_gotLastInput &&
        ((_lastInput < _setPoint && input >= _setPoint) ||
         (_lastInput > _setPoint && input <= _setPoint))) {
        if (_gotZeroCross) {
            Serial.print("TBH: zero cross detected, last output =");
            Serial.println(_lastZeroCrossIntTerm);
            _intTerm = (_intTerm + _lastZeroCrossIntTerm) / 2;
        } else {
            Serial.println("TBH: first zero cross detected");
            _gotZeroCross = true;
        }
        _lastZeroCrossIntTerm = _intTerm;
    }
    _lastInput = input;
    _gotLastInput = true;
    if (_intTerm < minIntTerm)
        _intTerm = 0;
    else if (_intTerm > maxIntTerm)
        _intTerm = 1;
    _output = _kp * err + _intTerm;
    if (_output < 0)
        _output = 0;
    else if (_output > 1)
        _output = 1;
    Serial.print("TBH: ki*1000 = ");
    Serial.print(_ki * 1000);
    Serial.print(", kp*1000 = ");
    Serial.print(_kp * 1000);
    Serial.print(", fScaled = ");
    Serial.print(fScaled);
    Serial.print(", t = ");
    Serial.print(t);
    Serial.print(", interval = ");
    Serial.println(_interval);

    Serial.print("TBH: input = ");
    Serial.print(input);
    Serial.print(", setPoint = ");
    Serial.print(_setPoint);
    Serial.print(", err = ");
    Serial.print(err);
    Serial.print(", intTerm = ");
    Serial.print(_intTerm);
    Serial.print(", output = ");
    Serial.println(_output);

    // log("TBH: ki = %f, kp = %f, fScaled = %f, t = %ld, interval = %ld", _ki, _kp, fScaled, t, _interval);
    // log("TBH: input = %f, setPoint = %f, err = %f, intTerm = %f, output = %f", input, _setPoint, err, _intTerm, _output);

    return true;
}
