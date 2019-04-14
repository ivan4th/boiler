#include "boiler.h"

namespace {
    const char* devName = "boiler";
    uint8_t tempBoardAddr[8] = { 0x28, 0x0e, 0x2a, 0x71, 0x07, 0x00, 0x00, 0x15 };
    uint8_t tempBoilerToTankAddr[8] = { 0x28, 0xff, 0xb6, 0xd6, 0x22, 0x17, 0x04, 0xab };
    uint8_t tempTankToBoilerAddr[8] = { 0x28, 0xff, 0x62, 0xf5, 0x22, 0x17, 0x04, 0x12 };
    uint8_t tempTankToHouseAddr[8] = { 0x28, 0xff, 0x0e, 0xdb, 0x22, 0x17, 0x04, 0x47 };
    uint8_t tempHouseToTankAddr[8] = { 0x28, 0xff, 0x80, 0x08, 0x23, 0x17, 0x04, 0x0c };
    uint8_t tempTankAAddr[8] = { 0x28, 0xff, 0xc0, 0x62, 0xa6, 0x16, 0x03, 0x7e };
    uint8_t tempTankBAddr[8] = { 0x28, 0xff, 0x6b, 0xa0, 0x24, 0x17, 0x03, 0xea };
    uint8_t tempTankCAddr[8] = { 0x28, 0xff, 0xaf, 0x3e, 0x81, 0x14, 0x02, 0xeb };

    static constexpr double pressureCoefA = 0.014705882352941176,
        pressureCoefB = -1.3970588235294117,
        minX = 65,
        maxX = 344,
        xCoefA = 100.0 / (maxX - minX),
        xCoefB = -65 * xCoefA;
};

Boiler::Boiler(Mqtt* mqtt, Eeprom* eeprom, BoardIO *io):
    _cellSet(mqtt, eeprom, devName),
    _binder(io),
    _tempTankToHouse(_cellSet.addCell<double>("temp-tank-to-house")),
    _tempHouseToTank(_cellSet.addCell<double>("temp-house-to-tank")),
    _tempBoard(_cellSet.addCell<double>("temp-board")),
    _tempBoilerToTank(_cellSet.addCell<double>("temp-boiler-to-tank")),
    _tempTankToBoiler(_cellSet.addCell<double>("temp-tank-to-boiler")),
    _tempTankA(_cellSet.addCell<double>("temp-tank-a")),
    _tempTankB(_cellSet.addCell<double>("temp-tank-b")),
    _tempTankC(_cellSet.addCell<double>("temp-tank-c")),
    // FIXME: should be valve-x, valve-y
    _valveX(_cellSet.addCell<double>("valveX")),
    _valveY(_cellSet.addCell<double>("valveY")->setWritable()),
    // FIXME: should be target-temp
    _targetTemp(_cellSet.addCell<double>("targetTemp")->setWritable()->setPersistent()),
    _kp(_cellSet.addCell<double>("kp")->setWritable()->setPersistent()),
    _ki(_cellSet.addCell<double>("ki")->setWritable()->setPersistent()),
    _kd(_cellSet.addCell<double>("kd")->setWritable()->setPersistent()),
    _pressure(_cellSet.addCell<double>("pressure")),
    _feedLowPressureThreshold(_cellSet.addCell<double>("feed-low-pressure-threshold")->setWritable()->setPersistent()),
    _feedHighPressureThreshold(_cellSet.addCell<double>("feed-high-pressure-threshold")->setWritable()->setPersistent()),
    _enableValveControl(_cellSet.addCell<bool>("enable-valve-control")->setWritable()->setPersistent()),
    // FIXME: "feed-valve-open"
    _feedValveOpen(_cellSet.addCell<bool>("feedValveOpen")->setWritable()),
    _enableFeedValveControl(_cellSet.addCell<bool>("enable-feed-valve-control")->setWritable()->setPersistent()),
    _io(io) {}

void Boiler::setup()
{
    _cellSet.load();
    // FIXME: load() should invoke the notifications for cells
    // or there should be common setup() thing (event?)
    _pid = new CellPID(_tempTankToHouse, _valveY, _targetTemp, _kp, _ki, _kd, _enableValveControl, CellPID::Direct, 5000, 0, 100, _io);
    _feedValveControl = new CellHysteresisControl(_pressure, _feedValveOpen, _feedLowPressureThreshold, _feedHighPressureThreshold, _enableFeedValveControl);
    
    _binder.addAnalogInputBinding(ValveXPin, _valveX, ValueTransform(xCoefA, xCoefB));
    _binder.addAnalogOutputBinding(ValveYPin, _valveY, ValueTransform(2.55, 0, 0, 255));
    _binder.addAnalogInputBinding(PressurePin, _pressure, ValueTransform(pressureCoefA, pressureCoefB));
    _binder.addTemperatureBinding(TemperaturePin, _tempTankToHouse, tempTankToHouseAddr, 10);
    _binder.addTemperatureBinding(TemperaturePin, _tempHouseToTank, tempHouseToTankAddr, 50);
    _binder.addTemperatureBinding(TemperaturePin, _tempBoard, tempBoardAddr, 50);
    _binder.addTemperatureBinding(TemperaturePin, _tempBoilerToTank, tempBoilerToTankAddr, 50);
    _binder.addTemperatureBinding(TemperaturePin, _tempTankToBoiler, tempTankToBoilerAddr, 50);
    _binder.addTemperatureBinding(TemperaturePin, _tempTankA, tempTankAAddr, 50);
    _binder.addTemperatureBinding(TemperaturePin, _tempTankB, tempTankBAddr, 50);
    _binder.addTemperatureBinding(TemperaturePin, _tempTankC, tempTankCAddr, 50);
    _binder.addDigitalOutputBinding(FeedValvePin, _feedValveOpen);
    _binder.setup();
}

void Boiler::loop()
{
    _binder.updateCells();
    _pid->compute();
    _feedValveControl->compute();
}

// TODO: enable valve control by default
