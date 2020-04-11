#ifndef __BOILER_H__
#define __BOILER_H__

#include "interfaces.h"
#include "cell.h"
#include "binder.h"
#include "cell_pid.h"
#include "cell_hysteresis.h"

class Boiler {
public:
    enum {
          TemperaturePin = 1,
          ValveXPin,
          ValveYPin,
          PressurePin,
          FeedValvePin
    };
    Boiler(Mqtt* mqtt, Eeprom* eeprom, BoardIO *io);
    void setup();
    void loop();
private:
    CellSet _cellSet;
    CellBinder _binder;
    TypedCell<double> *_tempTankToHouse,
        *_tempHouseToTank,
        *_tempBoard,
        *_tempBoilerToTank,
        *_tempTankToBoiler,
        *_tempTankA,
        *_tempTankB,
        *_tempTankC,
        *_valveX, *_valveY, *_targetTemp, *_kp, *_ki, *_kd,
        *_pressure,
        *_feedLowPressureThreshold,
        *_feedHighPressureThreshold,
        *_burnerTemperature;
    TypedCell<bool> *_enableValveControl, *_feedValveOpen, *_enableFeedValveControl;
    BoardIO* _io;
    CellPID* _pid;
    CellHysteresisControl* _feedValveControl;
};

#endif
