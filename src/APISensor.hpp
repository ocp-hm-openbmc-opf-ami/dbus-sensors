#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <string>
#include <vector>


// From DigitalDiscrete.hpp
#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

typedef int(*initFunction_T)(void);
typedef double(*readFunction_T)(void);
typedef int(*writeFunction_T)(double);

class APISensor :
    public Sensor,
    public std::enable_shared_from_this<APISensor>
{
  public:
    APISensor(
        const std::string& objectType,
        sdbusplus::asio::object_server& objectServer,
        boost::asio::io_context& io,
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        const std::string& sensorName,
        const std::string& sensorUnits,
        std::vector<thresholds::Threshold>&& thresholdsIn,
        const std::string& sensorConfiguration,
        double maxReading,
        double minReading,
        const PowerState& powerState,
        double sensorScale,
        const double sensorEventReadingType,
        const double sensorTypeCode,
        const double sensorEntityId,
        const double sensorEntityInstance,
        const std::string& sensorTypePathOverride,
        const std::string& sensorInitFuncName,
        const std::string& sensorReadFuncName,
        const std::string& sensorWriteFuncName,
        unsigned long sensorPollTimeMs,
        void *libHandle);
    ~APISensor() override;

    // Call this immediately after calling the constructor
    void initWriteHook(std::function<void()>&& writeHookIn);

    // Starts a timer that periodically calls performRead() (time based on pollTimeMs)
    void startMonitor(void);

    initFunction_T initFunction;
    readFunction_T readFunction;
    writeFunction_T writeFunction;

  private:
    std::function<void()> writeHook;
    // Obtains sensor reading from library using readFunction pointer
    // which was obtained from dlsym of libapisensor.so
    void performRead(void);
    void checkThresholds() override;
    void externalSetTrigger();
    std::string getSensorPathFromUnits(const std::string& sensorUnits,
                                       [[maybe_unused]] const double& entityId);

    const boost::container::flat_map<std::string, std::string> sensorUnitsToPathMap = {
        {"DegreesC", "temperature"},
        {"Volts", "voltage"},
        {"Meters", "altitude"},
        {"Amperes", "current"},
        {"Watts", "power"},
        {"Joules", "energy"},
        {"Percent", "utilization"},
        {"KPA", "pressurekpa"},
        {"LPM", "flowrate"},
        {"CFM", "airflow"},
        {"PWM", "pwm"},
        {"RPM", "tach"},
        {"Hours", "hours"},
        {"Count", "count"}
    };

    std::string objType;
    sdbusplus::asio::object_server& objServer;
    boost::asio::steady_timer monitorTimer;
    double scale;
    double eventReadingType;
    double typeCode;
    double entityId;
    double entityInstance;
    std::string typePathOverride;
    std::string initFuncName;
    std::string readFuncName;
    std::string writeFuncName;
    unsigned long pollTimeMs;
    void *libAPISensorHandle;
};


class APISensorDiscrete :
    public Discrete,
    public std::enable_shared_from_this<APISensorDiscrete>
{
  public:
    APISensorDiscrete(
          const std::string& objectType,
          sdbusplus::asio::object_server& objectServer,
          boost::asio::io_context& io,
          std::shared_ptr<sdbusplus::asio::connection>& conn,
          const std::string& sensorName,
          const std::string& sensorConfiguration,
          const PowerState& powerState,
          const double sensorEventReadingType,
          const double sensorTypeCode,
          const double sensorEntityId,
          const double sensorEntityInstance,
          const std::string& sensorTypePathOverride,
          const std::vector<std::string>& sensorStates,
          const std::string& sensorInitFuncName,
          const std::string& sensorReadFuncName,
          const std::string& sensorWriteFuncName,
          unsigned long sensorPollTimeMs,
          void *libHandle);

    ~APISensorDiscrete() override;

    // Call this immediately after calling the constructor
    void initWriteHook(std::function<void()>&& writeHookIn);

    // Starts a timer that periodically calls performRead() (time based on pollTimeMs)
    void startMonitor(void);

    initFunction_T initFunction;
    readFunction_T readFunction;
    writeFunction_T writeFunction;

  private:
    std::function<void()> writeHook;
    void externalSetTrigger();
    // Obtains sensor reading from library using readFunction pointer
    // which was obtained from dlsym of libapisensor.so
    void performRead(void);

    std::string objType;
    sdbusplus::asio::object_server& objServer;
    boost::asio::steady_timer monitorTimer;
    double eventReadingType;
    double typeCode;
    double entityId;
    double entityInstance;
    std::string typePathOverride;
    std::vector<std::string> states;
    std::string initFuncName;
    std::string readFuncName;
    std::string writeFuncName;
    unsigned long pollTimeMs;
    void *libAPISensorHandle;

    uint16_t laststate;


    // Discrete sensors with the matches below log deassertion events
    // { EventTypeCode, SensorTypeCode }
    std::vector<std::pair<uint8_t, uint8_t>> deassertionSensorTypes = {
        { 0x6f, 0x05 }, // Chassis Intrustion
        { 0x6f, 0x06 }, // Security Violation
        { 0x6f, 0x07 }, // Processor
        { 0x6f, 0x08 }, // Power Supply
        { 0x6f, 0x09 }, // Power Unit
        { 0x6f, 0x0c }, // Memory
        { 0x6f, 0x0d }, // Drive Bay
        { 0x6f, 0x10 }, // Event Logging
        { 0x6f, 0x11 }, // Watchdog
        { 0x6f, 0x13 }, // Citical Interript
        { 0x6f, 0x19 }, // Chip Set
        { 0x6f, 0x1b }, // Cable / Interconnect
        { 0x6f, 0x21 }, // Slot / Connector
        { 0x6f, 0x28 }, // Management Subsystem Health
        { 0x6f, 0x29 }  // Battery
    };
};

struct VariantToStringArrayVisitor
{
    template <typename T>
    std::vector<std::string> operator()(const T& t) const
    {
        if constexpr (std::is_same_v<T, std::vector<std::string>>)
        {
            return t;
        }
        throw std::invalid_argument(
            "Cannot translate type " +
            boost::typeindex::type_id<T>().pretty_name() + " to string array");
    }
};
