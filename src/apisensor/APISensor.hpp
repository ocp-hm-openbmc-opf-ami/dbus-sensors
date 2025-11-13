#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

typedef int (*initFunction_T)(std::string sensorName);
typedef double (*readFunction_T)(std::string sensorName);
typedef int (*writeFunction_T)(std::string sensorName, double);
using InfoVariantType = std::variant<bool, double, std::string>;
typedef int (*infoFunction_T)(std::string sensorName,
                              std::map<std::string, InfoVariantType>* mapPtr);
typedef int (*locationIndicatorFunction_T)(std::string sensorName, bool);

#define EVENT_SDR_TYPE 3

struct SetSensorInternalFailure : sdbusplus::exception_t
{
    const char* name() const noexcept override
    {
        return "xyz.openbmc_project.Common.Errors.InternalFailure";
    }
    const char* description() const noexcept override
    {
        return "Unable to set property value.";
    }
    int get_errno() const noexcept override
    {
        return EINVAL;
    }
};

class APISensor : public Sensor, public std::enable_shared_from_this<APISensor>
{
  public:
    APISensor(const std::string& objectType,
              sdbusplus::asio::object_server& objectServer,
              boost::asio::io_context& io,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              const std::string& sensorName, const std::string& sensorUnits,
              std::vector<thresholds::Threshold>&& thresholdsIn,
              const std::string& sensorConfiguration, double maxReading,
              double minReading, const PowerState& powerState,
              double sensorScale, const double sensorEventReadingType,
              const double sensorTypeCode, const double sensorEntityId,
              const double sensorEntityInstance,
              const std::string& sensorTypePathOverride,
              const std::string& sensorInitFuncName,
              const std::string& sensorReadFuncName,
              const std::string& sensorWriteFuncName,
              const std::string& sensorInfoFuncName,
              unsigned long sensorPollTimeMs, void* libHandle);
    ~APISensor() override;

    // Call this immediately after calling the constructor
    void initWriteHook(std::function<void()>&& writeHookIn);

    // Starts a timer that periodically calls performRead() (time based on
    // pollTimeMs)
    void startMonitor(void);

    initFunction_T initFunction;
    readFunction_T readFunction;
    writeFunction_T writeFunction;
    infoFunction_T infoFunction;

  private:
    std::function<void()> writeHook;
    // Obtains sensor reading from library using readFunction pointer
    // which was obtained from dlsym of libapisensor.so
    void performRead(void);
    void checkThresholds() override;
    void externalSetTrigger(double newValue);
    std::string getSensorPathFromUnits(const std::string& sensorUnits,
                                       [[maybe_unused]] const double& entityId);

    const boost::container::flat_map<std::string, std::string>
        sensorUnitsToPathMap = {
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
            {"Count", "count"},
            {"Hz", "frequency"}};
    std::string dbusSensorType;
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
    std::string infoFuncName;
    unsigned long pollTimeMs;
    void* libAPISensorHandle;
    std::map<std::string, InfoVariantType>
        infoPropertyMap; // Storage container for all info properties
    std::shared_ptr<sdbusplus::asio::dbus_interface> infoInterface;
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
        const std::string& sensorName, const std::string& sensorConfiguration,
        const PowerState& powerState, const double sensorEventReadingType,
        const double sensorTypeCode, const double sensorEntityId,
        const double sensorEntityInstance,
        const std::string& sensorTypePathOverride,
        const bool sensorProvidesProbeInterface,
        const std::vector<std::string>& sensorStates,
        const std::string& sensorInitFuncName,
        std::optional<uint8_t> sensorSDRType,
        const std::string& sensorReadFuncName,
        const std::string& sensorWriteFuncName,
        const std::string& sensorInfoFuncName,
        const std::string& sensorInfoFuncCallState,
        const std::string& sensorLocationIndicatorFuncName,
        unsigned long sensorPollTimeMs, void* libHandle);

    ~APISensorDiscrete() override;

    // Call this immediately after calling the constructor
    void initWriteHook(std::function<void()>&& writeHookIn);

    // Starts a timer that periodically calls performRead() (time based on
    // pollTimeMs)
    void startMonitor(void);

    initFunction_T initFunction;
    readFunction_T readFunction;
    writeFunction_T writeFunction;
    infoFunction_T infoFunction;
    locationIndicatorFunction_T locationIndicatorFunction;
    std::string configInterface; // This should really be moved to base class
                                 // (discrete.hpp)

  private:
    std::function<void()> writeHook;
    void externalSetTrigger(uint16_t newState);
    // Obtains sensor reading from library using readFunction pointer
    // which was obtained from dlsym of libapisensor.so
    void performRead(void);

    std::string dbusSensorType;
    std::string objType;
    sdbusplus::asio::object_server& objServer;
    boost::asio::steady_timer monitorTimer;
    double eventReadingType;
    double typeCode;
    double entityId;
    double entityInstance;
    std::string typePathOverride;
    std::shared_ptr<sdbusplus::asio::dbus_interface> probeInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> infoInterface;
    std::shared_ptr<sdbusplus::asio::dbus_interface> locationIndicatorInterface;
    bool providesProbeInterface;
    std::string probeName;  // Storage container for ProbeName property
    std::string probeState; // Storage container for ProbeState property
    std::vector<std::string> states;
    std::string initFuncName;
    std::string readFuncName;
    std::string writeFuncName;
    std::string infoFuncName;
    std::string infoFuncCallState;
    std::string locationIndicatorFuncName;
    bool locationIndicatorState;
    unsigned long pollTimeMs;
    void* libAPISensorHandle;

    uint16_t laststate;
    std::string objectPath;
    std::map<std::string, InfoVariantType>
        infoPropertyMap; // Storage container for all info properties

    // Discrete sensors with the matches below log deassertion events
    // { EventTypeCode, SensorTypeCode }
    std::vector<std::pair<uint8_t, uint8_t>> deassertionSensorTypes = {
        {0x6f, 0x05}, // Chassis Intrustion
        {0x6f, 0x06}, // Security Violation
        {0x6f, 0x07}, // Processor
        {0x6f, 0x08}, // Power Supply
        {0x6f, 0x09}, // Power Unit
        {0x6f, 0x0c}, // Memory
        {0x6f, 0x0d}, // Drive Bay
        {0x6f, 0x10}, // Event Logging
        {0x6f, 0x11}, // Watchdog
        {0x6f, 0x13}, // Citical Interript
        {0x6f, 0x19}, // Chip Set
        {0x6f, 0x1b}, // Cable / Interconnect
        {0x6f, 0x21}, // Slot / Connector
        {0x6f, 0x28}, // Management Subsystem Health
        {0x6f, 0x29}  // Battery
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

// TODO - Figure out a better way to dynamically register variant properties
inline int registerInfoPropertyVariant(
    std::string objType, std::string name, std::string prop,
    InfoVariantType val,
    std::shared_ptr<sdbusplus::asio::dbus_interface>& interface,
    std::map<std::string, InfoVariantType>& infoPropertyMap)
{
    int err = 0;
    std::size_t variantIndex = val.index();
    switch (variantIndex)
    {
        case 0: // bool
            try
            {
                interface->register_property_r(
                    prop, std::get<bool>(val),
                    sdbusplus::vtable::property_::emits_change,
                    [&infoPropertyMap, prop](bool& value) {
                        value = std::get<bool>(infoPropertyMap[prop]);
                        return value;
                    });
                break;
            }
            catch (std::exception& e)
            {
                std::cerr << objType << ": " << name
                          << ": error registering property: " << prop << ": "
                          << e.what() << std::endl;
                err = -1;
            }
        case 1: // double
            try
            {
                interface->register_property_r(
                    prop, std::get<double>(val),
                    sdbusplus::vtable::property_::emits_change,
                    [&infoPropertyMap, prop](double& value) {
                        value = std::get<double>(infoPropertyMap[prop]);
                        return value;
                    });
                break;
            }
            catch (std::exception& e)
            {
                std::cerr << objType << ": " << name
                          << ": error registering property: " << prop << ": "
                          << e.what() << std::endl;
                err = -1;
            }
        case 2: // std::string
            try
            {
                interface->register_property_r(
                    prop, std::get<std::string>(val),
                    sdbusplus::vtable::property_::emits_change,
                    [&infoPropertyMap, prop](std::string& value) {
                        value = std::get<std::string>(infoPropertyMap[prop]);
                        return value;
                    });
                break;
            }
            catch (std::exception& e)
            {
                std::cerr << objType << ": " << name
                          << ": error registering property: " << prop << ": "
                          << e.what() << std::endl;
                err = -1;
            }
        default:
            std::cerr << objType << ": " << name
                      << ": error registering property: " << prop
                      << ": unsupported variant type" << std::endl;
            err = -1;
    }
    return err;
}
