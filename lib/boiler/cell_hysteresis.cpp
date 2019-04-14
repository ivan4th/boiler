#include "cell_hysteresis.h"
#include "debug.h"

CellHysteresisControl::CellHysteresisControl(TypedCell<double>* inputCell,
                                             TypedCell<bool>* outputCell,
                                             TypedCell<double>* lowThresholdCell,
                                             TypedCell<double>* highThresholdCell,
                                             TypedCell<bool>* autoModeCell,
                                             bool inverse):
    _inputCell(inputCell),
    _outputCell(outputCell),
    _lowThresholdCell(lowThresholdCell),
    _highThresholdCell(highThresholdCell),
    _autoModeCell(autoModeCell),
    _inverse(inverse) {}

bool CellHysteresisControl::compute()
{
    if (_autoModeCell && !_autoModeCell->value())
        return false;

    double isOn = _inverse ? !_outputCell->value() : _outputCell->value();
    double inValue = _inputCell->value();
    if (isOn && inValue >= _highThresholdCell->value()) 
        _outputCell->setValue(_inverse);
    else if (!isOn && inValue <= _lowThresholdCell->value())
        _outputCell->setValue(!_inverse);
    else
        return false;

    return true;
}
