#include <unity.h>
#include <string.h>
#include <stdio.h>

#include "boiler.h"
#include "fake.h"

namespace {
    Recorder *rec = 0;
    FakeMqtt *mqtt = 0;
    FakeEeprom *eeprom = 0;
    FakeBoardIO *io = 0;
    CellSet *cellSet = 0;
    CellBinder *binder = 0;
    TypedCell<double> *doubleCell = 0, *roCell = 0;
    TypedCell<bool> *boolCell = 0;
};

void createCellSet()
{
    mqtt = new FakeMqtt(rec);
    cellSet = new CellSet(mqtt, eeprom, "testdev");
    binder = new CellBinder(io);
}

void createSimpleCells()
{
    doubleCell = cellSet->addCell<double>("double-cell", 42)->setWritable()->setPersistent();
    roCell = cellSet->addCell<double>("ro-cell", 1);
    boolCell = cellSet->addCell<bool>("bool-cell", false)->setWritable()->setPersistent();
}

void deleteCellSet()
{
    delete binder;
    delete cellSet;
    delete mqtt;
}

void setUp(void)
{
    rec = new Recorder();
    eeprom = new FakeEeprom();
    io = new FakeBoardIO(rec);
    createCellSet();
}

void tearDown(void)
{
    deleteCellSet();
    delete io;
    delete eeprom;
    delete rec;
    doubleCell = 0;
    roCell = 0;
    boolCell = 0;
}

void test_cells()
{
    createSimpleCells();
    rec->verify(REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/double-cell: 42.0000",
        "+/devices/testdev/controls/ro-cell: 1.0000",
        "+/devices/testdev/controls/bool-cell: 0",
        REC_END);

    mqtt->publish("/devices/testdev/controls/double-cell/on", "4242", false);
    rec->verify(
        "/devices/testdev/controls/double-cell/on: 4242",
        "+/devices/testdev/controls/double-cell: 4242.0000",
        REC_END);
    TEST_ASSERT_EQUAL_DOUBLE(4242, doubleCell->value());
    TEST_ASSERT_EQUAL(false, boolCell->value());

    mqtt->publish("/devices/testdev/controls/bool-cell/on", "1", false);
    rec->verify(
        "/devices/testdev/controls/bool-cell/on: 1",
        "+/devices/testdev/controls/bool-cell: 1",
        REC_END);
    TEST_ASSERT_EQUAL_DOUBLE(4242, doubleCell->value());
    TEST_ASSERT_EQUAL(true, boolCell->value());

    mqtt->publish("/devices/testdev/controls/bool-cell/on", "0", false);
    rec->verify(
        "/devices/testdev/controls/bool-cell/on: 0",
        "+/devices/testdev/controls/bool-cell: 0",
        REC_END);
    TEST_ASSERT_EQUAL_DOUBLE(4242, doubleCell->value());
    TEST_ASSERT_EQUAL(false, boolCell->value());

    // roCell value can't be set via MQTT
    mqtt->publish("/devices/testdev/controls/ro-cell/on", "0", false);
    rec->verify(
        "/devices/testdev/controls/ro-cell/on: 0",
        REC_END);
    TEST_ASSERT_EQUAL_DOUBLE(4242, doubleCell->value());
    TEST_ASSERT_EQUAL_DOUBLE(1, roCell->value());
    TEST_ASSERT_EQUAL(false, boolCell->value());

    doubleCell->setValue(42);
    rec->verify(
        "+/devices/testdev/controls/double-cell: 42.0000",
        REC_END);
    TEST_ASSERT_EQUAL_DOUBLE(42, doubleCell->value());

    boolCell->setValue(true);
    rec->verify(
        "+/devices/testdev/controls/bool-cell: 1",
        REC_END);
    TEST_ASSERT_EQUAL(true, boolCell->value());

    roCell->setValue(10);
    rec->verify(
        "+/devices/testdev/controls/ro-cell: 10.0000",
        REC_END);
    TEST_ASSERT_EQUAL(10, roCell->value());
}

void test_cell_storage()
{
    createSimpleCells();
    CellStorage storage(eeprom);
    TEST_ASSERT_FALSE(storage.startRead());

    TEST_ASSERT_FALSE(storage.startWrite());
    storage.writeBytes((const uint8_t*)"foo1", 4);
    storage.writeBytes((const uint8_t*)"z", 1);

    TEST_ASSERT_TRUE(storage.startRead());
    uint8_t buf[16];
    storage.readBytes(buf, 4);
    TEST_ASSERT(!memcmp(buf, "foo1", 4));
    storage.readBytes(buf, 1);
    TEST_ASSERT_EQUAL('z', buf[0]);

    TEST_ASSERT_TRUE(storage.startRead());
    storage.skip(4);
    storage.readBytes(buf, 1);
    TEST_ASSERT_EQUAL('z', buf[0]);

    TEST_ASSERT_TRUE(storage.startWrite());
    storage.skip(4);
    storage.writeBytes((const uint8_t*)"x", 1);

    TEST_ASSERT_TRUE(storage.startRead());
    storage.skip(4);
    storage.readBytes(buf, 1);
    TEST_ASSERT_EQUAL('x', buf[0]);
}

void test_persistent_cells()
{
    createSimpleCells();
    TEST_ASSERT_FALSE(cellSet->load());
    mqtt->connect();
    mqtt->publish("/devices/testdev/controls/double-cell/on", "4242", false);
    mqtt->publish("/devices/testdev/controls/bool-cell/on", "1", false);
    deleteCellSet();
    createCellSet();
    createSimpleCells();

    TEST_ASSERT_TRUE(cellSet->load());
    TEST_ASSERT_EQUAL_DOUBLE(4242, doubleCell->value());
    TEST_ASSERT_EQUAL_DOUBLE(1, roCell->value());
    TEST_ASSERT_EQUAL(true, boolCell->value());

    mqtt->connect();
    mqtt->publish("/devices/testdev/controls/double-cell/on", "4243", false);

    deleteCellSet();
    createCellSet();
    createSimpleCells();

    TEST_ASSERT_TRUE(cellSet->load());
    TEST_ASSERT_EQUAL_DOUBLE(4243, doubleCell->value());
    TEST_ASSERT_EQUAL_DOUBLE(1, roCell->value());
    TEST_ASSERT_EQUAL(true, boolCell->value());
}

void test_cell_digital_input_pin_bindings()
{
    TypedCell<bool>* inputCell =
        cellSet->addCell<bool>("in", false);
    binder->addDigitalInputBinding(13, inputCell);
    binder->setup();
    binder->updateCells();
    rec->verify(
        "Pin 13 mode Input",
        REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/in: 0",
        REC_END);

    io->setDigitalPinValue(13, BoardIO::High);

    // update interval not passed yet
    binder->updateCells();
    rec->verify(REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "+/devices/testdev/controls/in: 1",
        REC_END);

    io->setDigitalPinValue(13, BoardIO::Low);
    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "+/devices/testdev/controls/in: 0",
        REC_END);
}

void test_cell_digital_output_pin_bindings()
{
    TypedCell<bool>* switchCell =
        cellSet->addCell<bool>("switch", false)->setWritable()->setPersistent();
    binder->addDigitalOutputBinding(12, switchCell);
    binder->setup();
    binder->updateCells();
    rec->verify(
        "Pin 12 mode Output",
        "digitalWrite: 12 <- Low",
        REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/switch: 0",
        REC_END);
    binder->updateCells();
    rec->verify(REC_END);

    mqtt->publish("/devices/testdev/controls/switch/on", "1", false);
    rec->verify(
        "/devices/testdev/controls/switch/on: 1",
        "+/devices/testdev/controls/switch: 1",
        "digitalWrite: 12 <- High",
        REC_END);
    TEST_ASSERT_TRUE(switchCell->value());

    mqtt->publish("/devices/testdev/controls/switch/on", "0", false);
    rec->verify(
        "/devices/testdev/controls/switch/on: 0",
        "+/devices/testdev/controls/switch: 0",
        "digitalWrite: 12 <- Low",
        REC_END);
    TEST_ASSERT_FALSE(switchCell->value());
}

void test_cell_digital_output_pin_persistence()
{
    TypedCell<bool>* switchCell =
        cellSet->addCell<bool>("switch", false)->setWritable()->setPersistent();
    binder->addDigitalOutputBinding(12, switchCell);
    binder->setup();
    rec->verify(
        "Pin 12 mode Output",
        "digitalWrite: 12 <- Low",
        REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/switch: 0",
        REC_END);

    mqtt->publish("/devices/testdev/controls/switch/on", "1", false);
    rec->verify(
        "/devices/testdev/controls/switch/on: 1",
        "+/devices/testdev/controls/switch: 1",
        "digitalWrite: 12 <- High",
        REC_END);
    TEST_ASSERT_TRUE(switchCell->value());

    deleteCellSet();
    createCellSet();
    switchCell = cellSet->addCell<bool>("switch", false)->setWritable()->setPersistent();
    // NOTE: an important part is to load the CellSet before
    // adding the bindings.
    TEST_ASSERT_TRUE(cellSet->load());
    TEST_ASSERT_TRUE(switchCell->value());
    binder->addDigitalOutputBinding(12, switchCell);
    binder->setup();
    rec->verify(
        "Pin 12 mode Output",
        "digitalWrite: 12 <- High",
        REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/switch: 1",
        REC_END);

    mqtt->publish("/devices/testdev/controls/switch/on", "0", false);
    rec->verify(
        "/devices/testdev/controls/switch/on: 0",
        "+/devices/testdev/controls/switch: 0",
        "digitalWrite: 12 <- Low",
        REC_END);
    TEST_ASSERT_FALSE(switchCell->value());

    deleteCellSet();
    createCellSet();
    switchCell = cellSet->addCell<bool>("switch", false)->setWritable()->setPersistent();
    TEST_ASSERT_TRUE(cellSet->load());
    TEST_ASSERT_FALSE(switchCell->value());
    binder->addDigitalOutputBinding(12, switchCell);
    binder->setup();
    rec->verify(
        "Pin 12 mode Output",
        "digitalWrite: 12 <- Low",
        REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/switch: 0",
        REC_END);
}

void test_cell_analog_pin_bindings()
{
    TypedCell<double>* analogInputCell =
        cellSet->addCell<double>("ainput", 0);
    TypedCell<double>* analogOutputCell =
        cellSet->addCell<double>("aout", 10)->setWritable()->setPersistent();

    binder->addAnalogInputBinding(42, analogInputCell);
    binder->addAnalogOutputBinding(43, analogOutputCell, ValueTransform(10, 0, 0, 1000));
    binder->setup();
    io->setAnalogPinValue(42, 123);
    binder->updateCells();
    rec->verify(
        "Pin 43 mode Output",
        "analogWrite: 43 <- 100",
        "Pin 42 mode Input",
        REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/ainput: 123.0000",
        "+/devices/testdev/controls/aout: 10.0000",
        REC_END);

    // update interval not passed yet
    binder->updateCells();
    rec->verify(REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "+/devices/testdev/controls/ainput: 123.0000",
        REC_END);
    for (int i = 0; i < 3; ++i) {
        io->setAnalogPinValue(42, 1230);
        io->elapse(1000);
        binder->updateCells();
        rec->verify(
            "elapse: 1000",
            "+/devices/testdev/controls/ainput: 1230.0000",
            REC_END);
    }

    for (int i = 0; i < 3; ++i) {
        mqtt->publish("/devices/testdev/controls/aout/on", "25", false);
        rec->verify(
            "/devices/testdev/controls/aout/on: 25",
            "+/devices/testdev/controls/aout: 25.0000",
            "analogWrite: 43 <- 250",
            REC_END);
        TEST_ASSERT_EQUAL_DOUBLE(25, analogOutputCell->value());
    }
}

void test_cell_analog_pin_bindings_with_transform_and_averaging()
{
    TypedCell<double>* analogInputCell =
        cellSet->addCell<double>("ainput", 0);
    binder->addAnalogInputBinding(42, analogInputCell, ValueTransform(10, 3, 33, 20013), 3);
    binder->setup();
    io->setAnalogPinValue(42, 123);
    binder->updateCells();
    rec->verify(
        "Pin 42 mode Input",
        REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/ainput: 1233.0000",
        REC_END);

    io->setAnalogPinValue(42, 1230);
    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        // avg: 1233 12303
        "+/devices/testdev/controls/ainput: 6768.0000",
        REC_END);

    io->setAnalogPinValue(42, 6);
    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        // avg: 1233 12303 63
        "+/devices/testdev/controls/ainput: 4533.0000",
        REC_END);

    io->setAnalogPinValue(42, 18);
    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        // avg: 12303 63 183
        "+/devices/testdev/controls/ainput: 4183.0000",
        REC_END);

    // 1 -> 13 -> replaced by min 33
    io->setAnalogPinValue(42, 1);
    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        // avg: 63 183 33
        "+/devices/testdev/controls/ainput: 93.0000",
        REC_END);

    // 3000 -> 30003 -> replaced by max 20013
    io->setAnalogPinValue(42, 3000);
    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        // avg: 33 2013 20013
        "+/devices/testdev/controls/ainput: 6743.0000",
        REC_END);
}

void test_cell_temperature_bindings()
{
    TypedCell<double>* temp1 = cellSet->addCell<double>("temp1", 0);
    TypedCell<double>* temp2 = cellSet->addCell<double>("temp2", 0);
    static uint8_t addr1[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    static uint8_t addr2[8] = {1, 2, 3, 4, 5, 6, 7, 9};
    binder->addTemperatureBinding(11, temp1, addr1);
    binder->addTemperatureBinding(11, temp2, addr2);
    binder->setup();
    binder->updateCells();
    rec->verify("request", REC_END);
    mqtt->connect();
    rec->verify(REC_END);
    io->elapse(10);
    binder->updateCells();
    rec->verify("elapse: 10", REC_END);
    io->setTemperature(11, addr1, 23);
    io->setTemperature(11, addr2, 24);
    io->makeTempsAvailable();
    binder->updateCells();
    rec->verify(
        "+/devices/testdev/controls/temp2: 24.0000",
        "+/devices/testdev/controls/temp1: 23.0000",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);

    io->setTemperature(11, addr1, 23.5);
    io->setTemperature(11, addr2, 24.5);
    io->makeTempsAvailable();
    binder->updateCells();
    rec->verify(
        "+/devices/testdev/controls/temp2: 24.5000",
        "+/devices/testdev/controls/temp1: 23.5000",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);

    // request timeout
    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);
    io->makeTempsAvailable();
    binder->updateCells();
    rec->verify(
        "+/devices/testdev/controls/temp2: 24.5000",
        "+/devices/testdev/controls/temp1: 23.5000",
        REC_END);
}

void test_cell_temperature_avg()
{
    TypedCell<double>* temp = cellSet->addCell<double>("temp", 0);
    static uint8_t addr[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    binder->addTemperatureBinding(11, temp, addr, 3);
    binder->setup();
    binder->updateCells();
    rec->verify("request", REC_END);
    mqtt->connect();
    rec->verify(REC_END);
    io->elapse(10);
    binder->updateCells();
    rec->verify("elapse: 10", REC_END);
    io->setTemperature(11, addr, 23);
    io->makeTempsAvailable();
    binder->updateCells();
    rec->verify(
        "+/devices/testdev/controls/temp: 23.0000",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);

    io->setTemperature(11, addr, 25);
    io->makeTempsAvailable();
    binder->updateCells();
    rec->verify(
        "+/devices/testdev/controls/temp: 24.0000",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);

    io->setTemperature(11, addr, 27);
    io->makeTempsAvailable();
    binder->updateCells();
    rec->verify(
        "+/devices/testdev/controls/temp: 25.0000",
        REC_END);
}

void test_pid_control()
{
    TypedCell<double>* kp = cellSet->addCell<double>("kp", 1)->setWritable()->setPersistent();
    TypedCell<double>* ki = cellSet->addCell<double>("ki", 2)->setWritable()->setPersistent();
    TypedCell<double>* kd = cellSet->addCell<double>("kd", 0)->setWritable()->setPersistent();
    TypedCell<double>* temp = cellSet->addCell<double>("temp", 0);
    TypedCell<double>* output = cellSet->addCell<double>("output", 0)->setWritable();
    TypedCell<double>* setPoint = cellSet->addCell<double>("setPoint", 23)->setPersistent()->setWritable();
    TypedCell<bool>* autoMode = cellSet->addCell<bool>("autoMode", true)->setPersistent()->setWritable();
    static uint8_t addr[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    binder->addTemperatureBinding(11, temp, addr);
    binder->addAnalogOutputBinding(43, output);
    CellPID pid(temp, output, setPoint, kp, ki, kd, autoMode, CellPID::Direct, 1000, 0, 100, io);
    binder->setup();
    binder->updateCells();
    rec->verify(
        "Pin 43 mode Output",
        "analogWrite: 43 <- 0",
        "request",
        REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/kp: 1.0000",
        "+/devices/testdev/controls/ki: 2.0000",
        "+/devices/testdev/controls/kd: 0.0000",
        "+/devices/testdev/controls/output: 0.0000",
        "+/devices/testdev/controls/setPoint: 23.0000",
        "+/devices/testdev/controls/autoMode: 1",
        REC_END);

    io->setTemperature(11, addr, 23);
    io->makeTempsAvailable();
    binder->updateCells();
    rec->verify(
        "+/devices/testdev/controls/temp: 23.0000",
        REC_END);
    io->elapse(1000);
    TEST_ASSERT_TRUE(pid.compute());
    rec->verify(
        "elapse: 1000",
        "+/devices/testdev/controls/output: 0.0000",
        "analogWrite: 43 <- 0",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);
    io->setTemperature(11, addr, 22);
    io->makeTempsAvailable();
    binder->updateCells();
    TEST_ASSERT_TRUE(pid.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 22.0000",
        // 1 from P term + 2 from I term
        "+/devices/testdev/controls/output: 3.0000",
        "analogWrite: 43 <- 3",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);
    io->makeTempsAvailable();
    binder->updateCells();
    TEST_ASSERT_TRUE(pid.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 22.0000",
        // 1 from P term + 4 from I term
        "+/devices/testdev/controls/output: 5.0000",
        "analogWrite: 43 <- 5",
        REC_END);

    // Make sure that PID doesn't do anything once we turn off the auto mode
    mqtt->publish("/devices/testdev/controls/autoMode/on", "0", false);
    rec->verify(
        "/devices/testdev/controls/autoMode/on: 0",
        "+/devices/testdev/controls/autoMode: 0",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);
    io->makeTempsAvailable();
    binder->updateCells();
    TEST_ASSERT_FALSE(pid.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 22.0000",
        REC_END);
}

void test_hysteresis_control()
{
    TypedCell<double>* temp = cellSet->addCell<double>("temp", 0);
    TypedCell<bool>* output = cellSet->addCell<bool>("output", false)->setWritable();
    TypedCell<double>* lowThreshold = cellSet->addCell<double>("lowThreshold", 22)->setWritable()->setPersistent();
    TypedCell<double>* highThreshold = cellSet->addCell<double>("highThreshold", 24)->setWritable()->setPersistent();
    TypedCell<bool>* autoMode = cellSet->addCell<bool>("autoMode", true)->setPersistent()->setWritable();
    static uint8_t addr[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    binder->addTemperatureBinding(11, temp, addr);
    binder->addDigitalOutputBinding(12, output);
    CellHysteresisControl hc(temp, output, lowThreshold, highThreshold, autoMode);
    binder->setup();
    binder->updateCells();
    rec->verify(
        "Pin 12 mode Output",
        "digitalWrite: 12 <- Low",
        "request",
        REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/output: 0",
        "+/devices/testdev/controls/lowThreshold: 22.0000",
        "+/devices/testdev/controls/highThreshold: 24.0000",
        "+/devices/testdev/controls/autoMode: 1",
        REC_END);

    io->setTemperature(11, addr, 23);
    io->makeTempsAvailable();
    binder->updateCells();
    TEST_ASSERT_FALSE(hc.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 23.0000",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);
    io->setTemperature(11, addr, 21);
    io->makeTempsAvailable();
    binder->updateCells();
    TEST_ASSERT_TRUE(hc.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 21.0000",
        "+/devices/testdev/controls/output: 1",
        "digitalWrite: 12 <- High",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);
    io->setTemperature(11, addr, 25);
    io->makeTempsAvailable();
    binder->updateCells();
    TEST_ASSERT_TRUE(hc.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 25.0000",
        "+/devices/testdev/controls/output: 0",
        "digitalWrite: 12 <- Low",
        REC_END);

    // Make sure that the controller doesn't do anything once we turn off the auto mode
    mqtt->publish("/devices/testdev/controls/autoMode/on", "0", false);
    rec->verify(
        "/devices/testdev/controls/autoMode/on: 0",
        "+/devices/testdev/controls/autoMode: 0",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);
    io->makeTempsAvailable();
    io->setTemperature(11, addr, 21);
    binder->updateCells();
    TEST_ASSERT_FALSE(hc.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 21.0000",
        REC_END);
}

void test_hysteresis_control_inverse()
{
    TypedCell<double>* temp = cellSet->addCell<double>("temp", 0);
    TypedCell<bool>* output = cellSet->addCell<bool>("output", true)->setWritable();
    TypedCell<double>* lowThreshold = cellSet->addCell<double>("lowThreshold", 22)->setWritable()->setPersistent();
    TypedCell<double>* highThreshold = cellSet->addCell<double>("highThreshold", 24)->setWritable()->setPersistent();
    TypedCell<bool>* autoMode = cellSet->addCell<bool>("autoMode", true)->setPersistent()->setWritable();
    static uint8_t addr[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    binder->addTemperatureBinding(11, temp, addr);
    binder->addDigitalOutputBinding(12, output);
    CellHysteresisControl hc(temp, output, lowThreshold, highThreshold, autoMode, true);
    binder->setup();
    binder->updateCells();
    rec->verify(
        "Pin 12 mode Output",
        "digitalWrite: 12 <- High",
        "request",
        REC_END);
    mqtt->connect();
    rec->verify(
        "+/devices/testdev/controls/output: 1",
        "+/devices/testdev/controls/lowThreshold: 22.0000",
        "+/devices/testdev/controls/highThreshold: 24.0000",
        "+/devices/testdev/controls/autoMode: 1",
        REC_END);

    io->setTemperature(11, addr, 23);
    io->makeTempsAvailable();
    binder->updateCells();
    TEST_ASSERT_FALSE(hc.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 23.0000",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);
    io->setTemperature(11, addr, 21);
    io->makeTempsAvailable();
    binder->updateCells();
    TEST_ASSERT_TRUE(hc.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 21.0000",
        "+/devices/testdev/controls/output: 0",
        "digitalWrite: 12 <- Low",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);
    io->setTemperature(11, addr, 25);
    io->makeTempsAvailable();
    binder->updateCells();
    TEST_ASSERT_TRUE(hc.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 25.0000",
        "+/devices/testdev/controls/output: 1",
        "digitalWrite: 12 <- High",
        REC_END);

    // Make sure that the controller doesn't do anything once we turn off the auto mode
    mqtt->publish("/devices/testdev/controls/autoMode/on", "0", false);
    rec->verify(
        "/devices/testdev/controls/autoMode/on: 0",
        "+/devices/testdev/controls/autoMode: 0",
        REC_END);

    io->elapse(1000);
    binder->updateCells();
    rec->verify(
        "elapse: 1000",
        "request",
        REC_END);
    io->makeTempsAvailable();
    io->setTemperature(11, addr, 21);
    binder->updateCells();
    TEST_ASSERT_FALSE(hc.compute());
    rec->verify(
        "+/devices/testdev/controls/temp: 21.0000",
        REC_END);
}

void test_boiler()
{
    Boiler boiler(mqtt, eeprom, io);
    boiler.setup();
    boiler.loop();
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_cells);
    RUN_TEST(test_cell_storage);
    RUN_TEST(test_persistent_cells);
    RUN_TEST(test_cell_digital_input_pin_bindings);
    RUN_TEST(test_cell_digital_output_pin_bindings);
    RUN_TEST(test_cell_digital_output_pin_persistence);
    RUN_TEST(test_cell_analog_pin_bindings);
    RUN_TEST(test_cell_analog_pin_bindings_with_transform_and_averaging);
    RUN_TEST(test_cell_temperature_bindings);
    RUN_TEST(test_cell_temperature_avg);
    RUN_TEST(test_pid_control);
    RUN_TEST(test_hysteresis_control);
    RUN_TEST(test_hysteresis_control_inverse);
    RUN_TEST(test_boiler);
    UNITY_END();

    return 0;
}

// (setf irony-additional-clang-options
//  '("-std=c++11"
//    "-I/Users/ivan4th/work/newvas/boiler/lib/boiler"
//    "-I/Users/ivan4th/work/newvas/boiler/.piolibdeps/CONTROLLINO_ID397"
//    "-DTEST_NATIVE"))

// TODO: logging (with levels & MQTT option & MQTT level setting)
// TODO: log missing temp sensors
// TODO: Arduino impl for BoardIO / TempSensors / logging / etc.
// TODO: log available DS18B20 sensors on startup
// TODO: main loop with possibility to have cycle counter
//       and a way to log the number of cycles per second
// TODO: check that non-persistent cells aren't persisted (and aren't loaded!)
// TODO: specify storage version in CellSet constructor / through a method
// TODO: use mdX (or other simple hash) of persistent cell structure
//       AND version id
//       (e.g. https://github.com/tzikis/ArduinoMD5 https://github.com/scottmac/arduino/blob/master/md2/md2.pde)
// TODO: print available non-fragmented heap
// TODO: onSetup() event where all the cell bindings should initialize themselves, PID/hysteresis should initialize etc.
// TODO: adaptive PID. Params: kpStable / kiStable / kdStable, stableTime, stableMaxAbsDiff
// TODO: use good default PID values. kp=12.5, ki=0.08 faster, ki=0.04 stabler; kd=50
// TODO: ThresholdControl (set bool cell to true / false when another cell's value crosses the threshold)
//       To be used for the circulator pump and maybe for room temp
// TODO: feed valve open counter (with EEPROM storage)
