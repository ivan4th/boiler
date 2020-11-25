#include "circulation.h"
#include "debug.h"

namespace {
    const double MIN_TEMP = 10;
}

Circulation::Circulation(TypedCell<double>* burnerTempCell,
                         TypedCell<bool>* switchCell,
                         TypedCell<bool>* autoModeCell,
                         TypedCell<double>* burnDetectionTempCell,
                         TypedCell<double>* warmupRateCell,
                         TypedCell<double>* cooldownTempCell,
                         TypedCell<double>* burnDetectionTimeoutCell,
                         TypedCell<double>* tempDerivCell,
                         int sampleTime,
                         int numSamples,
                         TimeSource* timeSource):
    _burnerTempCell(burnerTempCell),
    _switchCell(switchCell),
    _autoModeCell(autoModeCell),
    _burnDetectionTempCell(burnDetectionTempCell),
    _warmupRateCell(warmupRateCell),
    _cooldownTempCell(cooldownTempCell),
    _burnDetectionTimeoutCell(burnDetectionTimeoutCell),
    _tempDerivCell(tempDerivCell),
    _sampleTime(sampleTime),
    _numSamples(numSamples),
    _timeSource(timeSource),
    _curNumSamples(0),
    _p(0),
    _temps(new double[numSamples])
{
    _lastTime = millis();
}

Circulation::~Circulation()
{
    delete[] _temps;
}

unsigned long Circulation::millis() const
{
    return _timeSource->millis();
}

bool Circulation::checkIfNeedCirculation(unsigned long ms, double& tempDiff, bool& tempDiffCalculated)
{
    double t = _burnerTempCell->value();

    // if the temp doesn't look right, skip it
    if (t < MIN_TEMP)
        return true;

    int cur = _p;
    _temps[_p++] = t;
    if (_p == _numSamples)
        _p = 0;
    // _cur is now the current entry and _p is an entry that was _numSamples ago

    if (++_curNumSamples < _numSamples)
        return t >= _burnDetectionTempCell->value() || t > _cooldownTempCell->value();

    tempDiff = _temps[cur] - _temps[_p];
    tempDiffCalculated = true;

    // hot enough to need circulation?
    if (t >= _burnDetectionTempCell->value()) {
        burnDetectTS = ms > 0 ? ms : 1;
        return true;
    }

    // temperature rise detected?
    if (tempDiff >= _warmupRateCell->value()) {
        burnDetectTS = ms > 0 ? ms : 1;
        return true;
    }

    // there was burn detected recently enough?
    if (burnDetectTS != 0 && (ms - burnDetectTS) / 1000 < _burnDetectionTempCell->value())
        return true;

    // the boiler has cooled down?
    if (t <= _cooldownTempCell->value())
        return false;

    // otherwise, leave it as-is
    return _switchCell->value();
}

bool Circulation::compute()
{
    if (_autoModeCell && !_autoModeCell->value())
        return false;

    unsigned long now = millis();
    unsigned long deltaT = (now - _lastTime);
    if (deltaT < _sampleTime)
        return false;
    _lastTime = now;

    bool changed = false, tempDiffCalculated = false;
    double tempDiff;
    bool v = checkIfNeedCirculation(now, tempDiff, tempDiffCalculated);
    if (_switchCell->value() != v) {
        _switchCell->setValue(v);
        changed = true;
    }

    if (tempDiffCalculated) {
        // always update the cell, even if the derivative value didn't change
        _tempDerivCell->setValue(tempDiff);
        changed = true;
    }
    return changed;
}
