#ifndef __CELL_PID_H__
#define __CELL_PID_H__

#include "cell.h"
#include "pid.h"

class CellPID: public PIDClient, public CellObserver {
public:
    enum Direction {
         Direct = DIRECT,
         Reverse = REVERSE
    };
    CellPID(TypedCell<double>* inputCell,
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
            TimeSource* timeSource);
    unsigned long millis() const;
    double input() const;
    double output() const;
    void setOutput(double newValue);
    double setPoint() const;
    void cellChanged(Cell*);
    bool compute();

private:
    TypedCell<double>* _inputCell;
    TypedCell<double>* _outputCell;
    TypedCell<double>* _setPointCell;
    TypedCell<double>* _kpCell;
    TypedCell<double>* _kiCell;
    TypedCell<double>* _kdCell;
    TypedCell<bool>* _autoModeCell;
    TimeSource* _timeSource;
    bool _started;
    PID _pid;
};

#endif
