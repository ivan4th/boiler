#ifndef __CELL_CIRCULATION_H__
#define __CELL_CIRCULATION_H__

#include "cell.h"

class Circulation {
public:
    Circulation(TypedCell<double>* burnerTempCell,
                TypedCell<bool>* switchCell,
                TypedCell<bool>* autoModeCell,
                TypedCell<double>* burnDetectionTempCell,
                TypedCell<double>* warmupRateCell,
                TypedCell<double>* cooldownTempCell,
                TypedCell<double>* burnDetectionTimeoutCell,
                TypedCell<double>* tempDerivCell,
                int sampleTime,
                int numSamples,
                TimeSource* timeSource);
    ~Circulation();
    unsigned long millis() const;
    bool compute();
private:
    bool checkIfNeedCirculation(unsigned long ms, double& tempDiff, bool& tempDiffCalculated);
    TypedCell<double>* _burnerTempCell;
    TypedCell<bool>* _switchCell;
    TypedCell<bool>* _autoModeCell;
    TypedCell<double>* _burnDetectionTempCell;
    TypedCell<double>* _warmupRateCell;
    TypedCell<double>* _cooldownTempCell;
    TypedCell<double>* _burnDetectionTimeoutCell;
    TypedCell<double>* _tempDerivCell;
    int _sampleTime;
    int _numSamples;
    TimeSource* _timeSource;
    int _curNumSamples;
    int _p;
    double* _temps;
    unsigned long _lastTime;
    unsigned long burnDetectTS;
};

#endif
