#ifndef __CELL_HYSTERESIS_H__
#define __CELL_HYSTERESIS_H__

#include "cell.h"

class CellHysteresisControl {
public:
    CellHysteresisControl(TypedCell<double>* inputCell,
                          TypedCell<bool>* outputCell,
                          TypedCell<double>* lowThresholdCell,
                          TypedCell<double>* highThresholdCell,
                          TypedCell<bool>* autoModeCell,
                          bool inverse = false);
    bool compute();
private:
    TypedCell<double>* _inputCell;
    TypedCell<bool>* _outputCell;
    TypedCell<double>* _lowThresholdCell;
    TypedCell<double>* _highThresholdCell;
    TypedCell<bool>* _autoModeCell;
    bool _inverse;
};

#endif
