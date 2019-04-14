#include "cell_pid.h"
#include "debug.h"

CellPID::CellPID(TypedCell<double>* inputCell,
                 TypedCell<double>* outputCell,
                 TypedCell<double>* setPointCell,
                 TypedCell<double>* kpCell,
                 TypedCell<double>* kiCell,
                 TypedCell<double>* kdCell,
                 TypedCell<bool>* autoModeCell,
                 Direction direction,
                 int sampleTime,
                 double minOutput,
                 double maxOutput,
                 TimeSource* timeSource):
    _inputCell(inputCell),
    _outputCell(outputCell),
    _setPointCell(setPointCell),
    _kpCell(kpCell),
    _kiCell(kiCell),
    _kdCell(kdCell),
    _autoModeCell(autoModeCell),
    _timeSource(timeSource),
    _started(false),
    _pid(this, kpCell->value(), kiCell->value(), kdCell->value(), P_ON_E, direction)
{
    _pid.SetSampleTime(sampleTime);
    _pid.SetOutputLimits(minOutput, maxOutput);
}

unsigned long CellPID::millis() const
{
    return _timeSource->millis();
}

double CellPID::input() const
{
    return _inputCell->value();
}

double CellPID::output() const
{
    return _outputCell->value();
}

void CellPID::setOutput(double newValue)
{
    _outputCell->setValue(newValue);
}

double CellPID::setPoint() const
{
    return _setPointCell->value();
}

void CellPID::cellChanged(Cell*)
{
    TRACE("cellChanged(): auto mode: %s", _autoModeCell->value() ? "true" : "false");
    _pid.SetTunings(_kpCell->value(), _kiCell->value(), _kdCell->value());
    if (_autoModeCell)
        _pid.SetMode(_autoModeCell->value() ? AUTOMATIC : MANUAL);
}

bool CellPID::compute()
{
    if (!_started) {
        _started = true;
        _kpCell->observe(this);
        _kiCell->observe(this);
        _kdCell->observe(this);
        if (_autoModeCell) {
            _autoModeCell->observe(this);
            _pid.SetMode(_autoModeCell->value() ? AUTOMATIC : MANUAL);
            TRACE("CellPID: init: auto mode: %s", _autoModeCell->value() ? "true" : "false");
        } else
            TRACE("CellPID: init: no auto mode cell");
    }

    return _pid.Compute();
}
