#ifndef __BINDINGS_H__
#define __BINDINGS_H__

#include "cell.h"
#include "average.h"

class ValueTransform {
public:
    ValueTransform(double a = 1, double b = 0, double min = -1e+9, double max = 1e+9):
        _a(a), _b(b), _min(min), _max(max) {}
    double apply(double v) const {
        v = v * _a + _b;
        if (v < _min)
            return _min;
        else if (v > _max)
            return _max;
        return v;
    }
private:
    double _a, _b, _min, _max;
};

class CellBinding: public CellObserver {
public:
    CellBinding(Cell* cell): _cell(cell) {
        _cell->observe(this);
    }
    virtual ~CellBinding() {}
    CellBinding* next() const { return _next; }
    void setNext(CellBinding* next) { _next = next; }
    Cell *cell() const { return _cell; }
    virtual void setup() = 0;
    virtual void updateCellFromTarget() = 0;
    virtual void updateTargetFromCell() = 0;
    virtual TempSensors* tempSensors() const { return 0; }
    void cellChanged(Cell* c) {
        updateTargetFromCell();
    }
    void setNotReady() {
        _cell->setNotReady();
    }
private:
    CellBinding* _next;
    Cell *_cell;
};

template<class T>
class TypedCellBinding: public CellBinding {
public:
    TypedCellBinding(TypedCell<T>* cell): CellBinding(cell) {}
    TypedCell<T> *cell() const {
        return static_cast<TypedCell<T>*>(CellBinding::cell());
    }
    virtual T getTargetValue() = 0;
    virtual void setTargetValue(T value) = 0;
    void updateCellFromTarget() { cell()->setValue(getTargetValue()); }
    void updateTargetFromCell() { setTargetValue(cell()->value()); }
private:
    Cell *_cell;
};

template<class T, BoardIO::PinMode Mode>
class PinBinding: public TypedCellBinding<T> {
public:
    PinBinding(BoardIO *io, int pinId, TypedCell<T> *cell):
        TypedCellBinding<T>(cell), _io(io), _pinId(pinId) {}
    void setup() {
        _io->setPinMode(_pinId, Mode);
    }
protected:
    BoardIO* _io;
    int _pinId;
};

template<class T, BoardIO::PinMode Mode>
class InputPinBinding: public PinBinding<T, Mode> {
public:
    InputPinBinding(BoardIO *io, int pinId, TypedCell<T> *cell):
        PinBinding<T, Mode>(io, pinId, cell) {}
    void setup() {
        PinBinding<T, Mode>::_io->setPinMode(PinBinding<T, Mode>::_pinId, BoardIO::Input);
    }
    void updateTargetFromCell() {}
    void setTargetValue(T value) {}
};

template<class T, BoardIO::PinMode Mode>
class OutputPinBinding: public PinBinding<T, Mode> {
public:
    OutputPinBinding(BoardIO *io, int pinId, TypedCell<T>* cell):
        PinBinding<T, Mode>(io, pinId, cell) {}
    void setup() {
        PinBinding<T, Mode>::_io->setPinMode(PinBinding<T, Mode>::_pinId, BoardIO::Output);
        PinBinding<T, Mode>::updateTargetFromCell();
    }
    void updateCellFromTarget() {}
    T getTargetValue() { return T(); }
};

class DigitalInputBinding: public InputPinBinding<bool, BoardIO::Input> {
public:
    DigitalInputBinding(BoardIO *io, int pinId, TypedCell<bool>* cell):
        InputPinBinding<bool, BoardIO::Input>(io, pinId, cell) {}
    bool getTargetValue() {
        return _io->digitalRead(_pinId) == BoardIO::High;
    }
};

class DigitalOutputBinding: public OutputPinBinding<bool, BoardIO::Output> {
public:
    DigitalOutputBinding(BoardIO *io, int pinId, TypedCell<bool>* cell, bool inverse = false):
        OutputPinBinding<bool, BoardIO::Output>(io, pinId, cell),
        _inverse(inverse) {}
    void setTargetValue(bool value) {
        _io->digitalWrite(_pinId, _inverse ?
                          (value ? BoardIO::Low : BoardIO::High) :
                          (value ? BoardIO::High : BoardIO::Low));
    }
private:
    bool _inverse;
};

class AnalogInputBinding: public InputPinBinding<double, BoardIO::Input> {
public:
    AnalogInputBinding(BoardIO *io, int pinId, TypedCell<double>* cell, const ValueTransform& transform = ValueTransform(), int nAvg = 1):
        InputPinBinding<double, BoardIO::Input>(io, pinId, cell),
        _transform(transform),
        _avg(nAvg) {}
    double getTargetValue() {
        _avg.add(_transform.apply(_io->analogRead(_pinId)));
        return _avg.value();
    }
private:
    ValueTransform _transform;
    RunningAverage _avg;
};

class AnalogOutputBinding: public OutputPinBinding<double, BoardIO::Output> {
public:
    AnalogOutputBinding(BoardIO *io, int pinId, TypedCell<double>* cell, const ValueTransform& transform = ValueTransform()):
        OutputPinBinding<double, BoardIO::Output>(io, pinId, cell),
        _transform(transform) {}
    void setTargetValue(double value) {
        _io->analogWrite(_pinId, _transform.apply(value));
    }
private:
    ValueTransform _transform;
};

class TemperatureBinding: public TypedCellBinding<double> {
public:
    TemperatureBinding(TempSensors* sensors, TypedCell<double>* cell, uint8_t* address, int nAvg = 1):
        TypedCellBinding<double>(cell),
        _sensors(sensors),
        _avg(nAvg) {
        setNotReady();
        memcpy(_address, address, TempSensors::AddressSize);
    }
    void setup() {}
    double getTargetValue() {
        _avg.add(_sensors->read(_address));
        return _avg.value();
    }
    void setTargetValue(double value) {}
    TempSensors* tempSensors() const { return _sensors; }

private:
    TempSensors* _sensors;
    RunningAverage _avg;
    uint8_t _address[TempSensors::AddressSize];
};

class ThermocoupleBinding: public TypedCellBinding<double> {
public:
    ThermocoupleBinding(BoardIO *io, TypedCell<double>* cell, int nAvg = 1):
        TypedCellBinding<double>(cell),
        _io(io),
        _avg(nAvg) {}
    void setup() {}
    void updateTargetFromCell() {}
    void setTargetValue(double value) {}
    double getTargetValue() {
        _avg.add(_io->getThermocoupleValue());
        return _avg.value();
    }
private:
    BoardIO* _io;
    RunningAverage _avg;
};

class CellBinder {
public:
    CellBinder(BoardIO* io, unsigned long tempPollIntervalMs = 1000, unsigned long pollIntervalMs = 1000);
    ~CellBinder();
    void addDigitalInputBinding(int pinId, TypedCell<bool>* cell);
    void addDigitalOutputBinding(int pinId, TypedCell<bool>* cell, bool inverse = false);
    void addAnalogInputBinding(int pinId, TypedCell<double>* cell, const ValueTransform& transform = ValueTransform(), int nAvg = 1);
    void addAnalogOutputBinding(int pinId, TypedCell<double>* cell, const ValueTransform& transform = ValueTransform());
    void addTemperatureBinding(int pinId, TypedCell<double>* cell, uint8_t* address, int nAvg = 1);
    void addThermocoupleBinding(TypedCell<double>* cell, int nAvg = 1);
    void setup();
    void updateCells();

private:
    static const int MaxTempSensors = 8;
    void addBinding(CellBinding* binding);
    enum TempPollState { NotStarted, TempsRequested, Wait };
 
    BoardIO* _io;
    CellBinding* _firstBinding;
    bool _nonTempsUpdated;
    unsigned long _tempPollIntervalMs, _pollIntervalMs, _tempsRequestedAt, _nonTempsUpdatedAt;
    TempPollState _tempPollState;
    TempSensors* _tempSensors;
};

#endif
