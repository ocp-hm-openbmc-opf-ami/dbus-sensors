#include "APISensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <dlfcn.h>
#include <sys/stat.h>
#include <unistd.h>

#include <boost/algorithm/string.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#define DISCRETE_SENSOR_SEL_LOGGING // TODO: move to meson.options

static constexpr bool debug = false;
static const std::string debugMsgPrefix = "APISensor: ";

APISensor::APISensor(
    const std::string& objectType, sdbusplus::asio::object_server& objectServer,
    boost::asio::io_context& io,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& sensorName, const std::string& sensorUnits,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const std::string& sensorConfiguration, double maxReading,
    double minReading, [[maybe_unused]] const PowerState& powerState,
    double sensorScale, [[maybe_unused]] const double sensorEventReadingType,
    [[maybe_unused]] const double sensorTypeCode,
    [[maybe_unused]] const double sensorEntityId,
    [[maybe_unused]] const double sensorEntityInstance,
    const std::string& sensorTypePathOverride,
    const std::string& sensorInitFuncName,
    const std::string& sensorReadFuncName,
    const std::string& sensorWriteFuncName,
    const std::string& sensorInfoFuncName, unsigned long sensorPollTimeMs,
    void* libHandle) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectType, true, true, maxReading, minReading, conn, powerState),
    objType(objectType), objServer(objectServer), monitorTimer(io),
    scale(sensorScale), eventReadingType(sensorEventReadingType),
    typeCode(sensorTypeCode), entityId(sensorEntityId),
    entityInstance(sensorEntityInstance),
    typePathOverride(sensorTypePathOverride), initFuncName(sensorInitFuncName),
    readFuncName(sensorReadFuncName), writeFuncName(sensorWriteFuncName),
    infoFuncName(sensorInfoFuncName), pollTimeMs(sensorPollTimeMs),
    libAPISensorHandle(libHandle)
{
    std::string dbusPath = getSensorPathFromUnits(sensorUnits, entityId);
    if (dbusPath.empty())
    {
        throw std::runtime_error("Units not in allow list");
    }
    dbusSensorType = dbusPath;

    // If sensor has an InitFunction, get function pointer
    if (!initFuncName.empty())
    {
        // Get pointer to InitFunction
        dlerror(); // First, clear any existing dlerror
        initFunction =
            (initFunction_T)dlsym(libAPISensorHandle, initFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error || initFunction == nullptr)
        {
            initFunction = nullptr;
            std::cerr << objType << ": " << name
                      << " Cannot find function symbol: " << initFuncName
                      << " - " << dlsym_error << "\n";
            // InitFunction is optional, continue on without it
        }
    }

    // If sensor has an InfoFunction, get function pointer
    if (!infoFuncName.empty())
    {
        // Get pointer to InfoFunction
        dlerror(); // Clear any existing error
        infoFunction =
            (infoFunction_T)dlsym(libAPISensorHandle, infoFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error)
        {
            infoFunction = nullptr;
            std::cerr << objType << ": " << name
                      << " Cannot find function symbol: " << infoFuncName
                      << " - " << dlsym_error << "\n";
            // Do not close the handle (it's needed for other sensors)
        }
    }

    // If ReadFunction is missing, sensor will always
    // report nan as the reading value.
    // However, ReadFunction is mandatory which means
    // the code may never reach this point if APISensorMain
    // skips creating sensors that are missing ReadFunctions.
    if (readFuncName.empty())
    {
        if constexpr (debug)
        {
            std::cerr << debugMsgPrefix << name << " ReadFunction missing!\n";
        }
    }
    else
    {
        // Get pointer to ReadFunction
        dlerror(); // First, clear any existing dlerror
        readFunction =
            (readFunction_T)dlsym(libAPISensorHandle, readFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error || (readFunction == nullptr))
        {
            std::cerr << debugMsgPrefix << name
                      << " Cannot find function symbol: " << readFuncName
                      << " - " << dlsym_error << "\n";
            // ReadFunction not found, ReadFunction is mandatory to obtain
            // sensor readings. Without it, the sensor will always report NaN as
            // the value
        }
        else
        {
            // Get initial value
            value = readFunction(name);
        }
    }

    // WriteFunction is optional
    if (!writeFuncName.empty())
    {
        // Get pointer to WriteFunction
        dlerror(); // First, clear any existing dlerror
        writeFunction =
            (writeFunction_T)dlsym(libAPISensorHandle, writeFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error || (writeFunction == nullptr))
        {
            std::cerr << debugMsgPrefix << name
                      << " Cannot find function symbol: " << writeFuncName
                      << " - " << dlsym_error << "\n";
            // WriteFunction is optional, continue on without it
        }
    }

    std::string objectPath = "/xyz/openbmc_project/sensors/";
    objectPath += typePathOverride.empty() ? dbusPath : typePathOverride;
    objectPath += '/';
    objectPath += escapeName(sensorName);

    // Call any potential InitFunction and/or InfoFunction but
    // post (defer) the calls to avoid blocking the constructor
    boost::asio::post(io, [this, objectPath]() {
        // If we have an InitFunction
        if (!initFuncName.empty() && initFunction != nullptr)
        {
            try
            {
                // Call the InitFunction
                if (initFunction(name) < 0)
                {
                    std::cerr << objType << ": " << name << ": InitFunction ("
                              << initFuncName << ") failed!\n";
                }
            }
            catch (std::exception& e)
            {
                std::cerr << objType << ": " << name << ": InitFunction ("
                          << initFuncName << ") failed! " << e.what() << "\n";
            }
        }

        // If we have an InfoFunction
        if (!infoFuncName.empty() && infoFunction != nullptr)
        {
            // Call InfoFunction to get custom info map
            std::map<std::string, InfoVariantType> info;
            int infoFuncError;
            try
            {
                infoFuncError = infoFunction(name, &info);
            }
            catch (std::exception& e)
            {
                std::cerr << objType << ": " << name << ": InfoFunction ("
                          << infoFuncName << ") failed! " << e.what()
                          << std::endl;
            }

            if (!infoFuncError)
            {
                // InfoFunction was successful, if info data returned
                if (info.size())
                {
                    // Create info interface
                    infoInterface = objServer.add_interface(
                        objectPath, "xyz.openbmc_project.Sensor.Info");

                    // For each key/value pair returned from InfoFunction
                    for (auto [key, val] : info)
                    {
                        if constexpr (debug)
                        {
                            std::cout << objType << ": " << name
                                      << ": Creating info property " << key
                                      << std::endl;
                        }
                        infoPropertyMap[key] = val; // Update info map
                        // Register key/value pair as dbus property/value
                        registerInfoPropertyVariant(objType, name, key, val,
                                                    infoInterface,
                                                    infoPropertyMap);
                    }
                    // Initialize Info interface
                    if (!infoInterface->initialize())
                    {
                        std::cerr << objType << ": " << name
                                  << ": error initializing info interface\n";
                    }
                }
            }
            else
            {
                std::cerr << objType << ": " << name << ": InfoFunction ("
                          << infoFuncName << ") failed!" << std::endl;
            }
        }
    });

    sensorInterface = objectServer.add_interface(
        objectPath, "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(objectPath, interface);
    }

    association =
        objectServer.add_interface(objectPath, association::interface);
    setInitialProperties(sensorUnits);

    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << name << " constructed APISensor path "
                  << configurationPath << ", readfunc " << readFuncName
                  << std::endl;
    }
}

APISensor::~APISensor()
{
    monitorTimer.cancel();

    // Make sure the write hook does not reference this object anymore
    externalSetHook = nullptr;

    objServer.remove_interface(association);
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);

    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << name << " destructed\n";
    }
}

std::string APISensor::getSensorPathFromUnits(
    const std::string& sensorUnits, [[maybe_unused]] const double& entityId)
{
    // Passing entityId but, currently have no need for it.

    std::string dbusPath;
    for (auto& [key, value] : sensorUnitsToPathMap)
    {
        if (sensorUnits == key)
        {
            dbusPath = value;
            break;
        }
    }
    return dbusPath;
}

// Separate function from constructor, because of a gotcha: can't use the
// enable_shared_from_this() API until after the constructor has completed.
void APISensor::initWriteHook(std::function<void()>&& writeHookIn)
{
    // Connect APISensorMain with APISensor if callback is provided
    if (writeHookIn != nullptr)
    {
        writeHook = std::move(writeHookIn);
    }

    // Connect APISensor with Sensor
    auto weakThis = weak_from_this();
    externalSetHook = [weakThis](double newValue) {
        auto lockThis = weakThis.lock();
        if (lockThis)
        {
            lockThis->externalSetTrigger(newValue);
            return;
        }
        if constexpr (debug)
        {
            std::cerr << debugMsgPrefix << "receive ignored, sensor gone\n";
        }
    };
}

void APISensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

// Invoked by base class (sensor.hpp) when
// sensor dbus value is updated externally
void APISensor::externalSetTrigger(double newValue)
{
    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << name << " dbus value externally set to "
                  << newValue << "\n";
    }

    // External source modified sensor (dbus).  Check if we have
    // a WriteFunction. If so, call it to update hardware.
    if (writeFunction != nullptr)
    {
        if (0 != writeFunction(name, newValue))
        {
            std::cout << debugMsgPrefix << name << " WriteFunction failed!\n";
            // externalSetTrigger is called from within the dbus Set properties
            // command handler. Throwing an exception here should propagate
            // back to the dbus caller.
            throw SetSensorInternalFailure();
        }
    }

    // Regardless of whether we have a WriteFunction, we must set
    // overriddenState to false to ensure data returned from ReadFunction
    // is updated to dbus.
    // Take back control of this sensor from the external override
    overriddenState = false; // defined and used in sensor.hpp (base class)
}

void APISensor::performRead(void)
{
    double reading;

    if (!readingStateGood())
    {
        markAvailable(false);
        startMonitor();
        return;
    }

    try
    {
        if (readFunction != nullptr)
        {
            reading = readFunction(name);

            struct stat file_info;
            bool debugShowReading =
                (0 == stat("/var/debugapisensor", &file_info));

            if (std::isnan(reading))
            {
                if (debugShowReading)
                {
                    std::cout << debugMsgPrefix << readFuncName
                              << " returned: NaN" << std::endl;
                }
                updateValue(std::numeric_limits<double>::quiet_NaN());
            }
            else
            {
#if 0 // Option to reduce reading to 2 decimal places
                double multiplier = std::pow(10, 2);
                reading = std::round(reading * multiplier) / multiplier;
#endif
                double scaledReading =
                    reading * scale; // Multiple reading by ScaleFactor
                if (debugShowReading)
                {
                    std::cout
                        << debugMsgPrefix << readFuncName
                        << " returned: " << reading
                        << ", scaledReading: " << scaledReading << std::endl;
                }

                // Take back control of this sensor incase
                // the reading was set externally via dbus.
                overriddenState = false;

                updateValue(scaledReading); // Update dbus value
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << debugMsgPrefix << "Exception in performRead() calling "
                  << readFuncName << "(): " << e.what() << std::endl;
    }
    startMonitor();
}

void APISensor::startMonitor(void)
{
    std::weak_ptr<APISensor> weakRef = weak_from_this();
    monitorTimer.expires_after(std::chrono::milliseconds(pollTimeMs));
    monitorTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        std::shared_ptr<APISensor> self = weakRef.lock();
        if (ec == boost::asio::error::operation_aborted)
        {
            // This is normal when object is removed - no need to SPAM
            // std::cerr << debugMsgPrefix << "Sensor monitor aborted";
            return;
        }
        if (self)
        {
            self->performRead();
        }
    });
}

APISensorDiscrete::APISensorDiscrete(
    const std::string& objectType, sdbusplus::asio::object_server& objectServer,
    boost::asio::io_context& io,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& sensorName, const std::string& sensorConfiguration,
    [[maybe_unused]] const PowerState& powerState,
    [[maybe_unused]] const double sensorEventReadingType,
    [[maybe_unused]] const double sensorTypeCode,
    [[maybe_unused]] const double sensorEntityId,
    [[maybe_unused]] const double sensorEntityInstance,
    const std::string& sensorTypePathOverride,
    const bool sensorProvidesProbeInterface,
    const std::vector<std::string>& sensorStates,
    const std::string& sensorInitFuncName,
    const std::string& sensorReadFuncName,
    const std::string& sensorWriteFuncName,
    const std::string& sensorInfoFuncName,
    const std::string& sensorInfoFuncCallState,
    const std::string& sensorLocationIndicatorFuncName,
    unsigned long sensorPollTimeMs, void* libHandle) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objType(objectType), objServer(objectServer), monitorTimer(io),
    eventReadingType(sensorEventReadingType), typeCode(sensorTypeCode),
    entityId(sensorEntityId), entityInstance(sensorEntityInstance),
    typePathOverride(sensorTypePathOverride),
    providesProbeInterface(sensorProvidesProbeInterface),
    probeName(escapeName(sensorName)), states(sensorStates),
    initFuncName(sensorInitFuncName), readFuncName(sensorReadFuncName),
    writeFuncName(sensorWriteFuncName), infoFuncName(sensorInfoFuncName),
    infoFuncCallState(sensorInfoFuncCallState),
    locationIndicatorFuncName(sensorLocationIndicatorFuncName),
    pollTimeMs(sensorPollTimeMs), libAPISensorHandle(libHandle)
{
    dbusSensorType = "discrete";
    configInterface = configInterfaceName(objectType);

    // If sensor has an InitFunction, get function pointer and call it
    if (!initFuncName.empty())
    {
        // Get pointer to InitFunction
        dlerror(); // First, clear any existing dlerror
        initFunction =
            (initFunction_T)dlsym(libAPISensorHandle, initFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error || (initFunction == nullptr))
        {
            initFunction = nullptr;
            std::cerr << debugMsgPrefix << name
                      << " Cannot find function symbol: " << initFuncName
                      << " - " << dlsym_error << "\n";
            // InitFunction is optional, continue on without it
        }
        else
        {
            // We have a valid InitFunction, let's call it.

            // Post (defer) init function to avoid blocking the constructor
            boost::asio::post(io, [this]() {
                // Call the init function
                int initFuncErrorCode = initFunction(name);
                if (initFuncErrorCode < 0)
                {
                    std::cerr << debugMsgPrefix << name
                              << " Error calling init function " << initFuncName
                              << ": " << initFuncErrorCode << "\n";
                    // InitFunction failed! Sensor readings may be incorrect.
                }
            });
        }
    }

    // If ReadFunction is missing, sensor will not return readings!
    if (readFuncName.empty())
    {
        if constexpr (debug)
        {
            std::cerr << debugMsgPrefix << name << " ReadFunction missing!\n";
        }
    }
    else
    {
        // Get pointer to ReadFunction
        dlerror(); // Clear any existing error
        readFunction =
            (readFunction_T)dlsym(libAPISensorHandle, readFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error)
        {
            readFunction = nullptr;
            std::cerr << debugMsgPrefix << name
                      << " Cannot find function symbol: " << readFuncName
                      << " - " << dlsym_error << "\n";
            // Do not close the handle (it's needed for other sensors)
        }
        else
        {
            // Get initial state
            state = readFunction(name);
        }
    }

    // WriteFunction is optional
    if (!writeFuncName.empty())
    {
        // Get pointer to WriteFunction
        dlerror(); // Clear any existing error
        writeFunction =
            (writeFunction_T)dlsym(libAPISensorHandle, writeFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error)
        {
            writeFunction = nullptr;
            std::cerr << debugMsgPrefix << name
                      << " Cannot find function symbol: " << writeFuncName
                      << " - " << dlsym_error << "\n";
            // Do not close the handle (it's needed for other sensors)
        }
    }

    // InfoFunction is optional
    if (!infoFuncName.empty())
    {
        // Get pointer to InfoFunction
        dlerror(); // Clear any existing error
        infoFunction =
            (infoFunction_T)dlsym(libAPISensorHandle, infoFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error)
        {
            infoFunction = nullptr;
            std::cerr << debugMsgPrefix << name
                      << " Cannot find function symbol: " << infoFuncName
                      << " - " << dlsym_error << "\n";
            // Do not close the handle (it's needed for other sensors)
        }
    }

    // Setup the dbus path for the sensor
    std::string sensorType = "discrete";
    objectPath = "/xyz/openbmc_project/sensors/";
    objectPath += typePathOverride.empty() ? sensorType : typePathOverride;
    objectPath += '/';
    objectPath += escapeName(sensorName);

    // LocationIndicatorFunction is optional
    if (!locationIndicatorFuncName.empty())
    {
        // Get pointer to LocationIndicatorFunction
        dlerror(); // Clear any existing error
        locationIndicatorFunction = (locationIndicatorFunction_T)dlsym(
            libAPISensorHandle, locationIndicatorFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error)
        {
            locationIndicatorFunction = nullptr;
            std::cerr << debugMsgPrefix << name
                      << " Cannot find function symbol: "
                      << locationIndicatorFuncName << " - " << dlsym_error
                      << "\n";
            // Do not close the handle (it's needed for other sensors)
        }
        else
        {
            // We have a valid LocationIndicatorFunction so let's call the
            // LocationIndicatorFunction to initialize indicator state to off
            // but, defer (post) the call to avoid blocking the constructor.
            boost::asio::post(io, [this]() {
                if (locationIndicatorFunction(name, false) < 0)
                {
                    std::cerr << debugMsgPrefix << name << " "
                              << locationIndicatorFuncName << "() - Failed!\n";
                }
            });

            // Create LocationIndicator interface and property
            locationIndicatorInterface = objectServer.add_interface(
                objectPath, "xyz.openbmc_project.Sensor.LocationIndicator");

            locationIndicatorState = false; // Storage container for property
            locationIndicatorInterface->register_property(
                "LocationIndicatorActive", locationIndicatorState,
                [this](const bool& newvalue, bool& oldvalue) {
                    oldvalue = locationIndicatorState;
                    locationIndicatorState = newvalue;
                    if (oldvalue != newvalue)
                    {
                        // Call LocationIndicatorFunction (set location
                        // indicator on/off)
                        if (locationIndicatorFunction(
                                name, locationIndicatorState) < 0)
                        {
                            std::cerr << debugMsgPrefix << name << " "
                                      << locationIndicatorFuncName
                                      << "() - Failed!\n";
                        }
                    }
                    return 1;
                },
                [this](bool& value) {
                    value = locationIndicatorState;
                    return value;
                });

            if (!locationIndicatorInterface->initialize())
            {
                std::cerr
                    << debugMsgPrefix << name
                    << " error initializing LocationIndicator interface\n";
            }
        }
    }

    sensorInterface = objectServer.add_interface(
        objectPath, "xyz.openbmc_project.Sensor.State");

    association =
        objectServer.add_interface(objectPath, association::interface);

    setInitialProperties();

    // "ProvidesProbeInterface": "Yes" | "No" is optional
    //
    // If "Yes", the sensor provides the below interface and properties
    // xyz.openbmc.project.Sensor.Probe
    //    ProbeName : <string>
    //    ProbeState : <string>
    // Example:
    //    ProbeName : "Fan1Present"
    //    ProbeState : "Present" | "Absent" | "Disabled"
    //
    // ProbeState is just the sensor reading in string format
    //
    // This feature can be used by entity-manager entities to probe on
    // a particular sensor and sensor state.
    // For example the below entity Probe returns true if sensor Fan1Present
    // state = Present:
    //     "Probe": [
    //         "xyz.openbmc_project.Sensor.Probe({'ProbeName': 'Fan1Present'},
    //         {'ProbeState', 'Present'})",
    //     ],
    //
    // Example use case is an entity presence sensor.
    //
    probeState = "na";
    if (sensorProvidesProbeInterface && !probeName.empty())
    {
        probeInterface = objectServer.add_interface(
            objectPath, "xyz.openbmc_project.Sensor.Probe");

        probeInterface->register_property("ProbeName", probeName);

        probeInterface->register_property(
            "ProbeState", probeState,
            [this](const std::string& newvalue, std::string& oldvalue) {
                oldvalue = probeState;
                probeState = newvalue;
                return 1;
            },
            [this](std::string& value) {
                value = probeState;
                return value;
            });

        if (!probeInterface->initialize())
        {
            std::cerr << debugMsgPrefix << name
                      << " error initializing probe interface\n";
        }
    }

    laststate = 0;

    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << name
                  << " constructed: LibAPISensorDiscrete path "
                  << configurationPath << ", type " << objectType
                  << ", readfunc " << readFuncName << ", writefunc "
                  << writeFuncName << ", infofunc " << infoFuncName
                  << ", probeIntf " << providesProbeInterface << "\n";
    }
}

APISensorDiscrete::~APISensorDiscrete()
{
    monitorTimer.cancel();

    // Make sure the write hook does not reference this object anymore
    externalSetHook = nullptr;

    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
    if (!probeName.empty())
    {
        objServer.remove_interface(probeInterface);
    }
}

void APISensorDiscrete::startMonitor(void)
{
    std::weak_ptr<APISensorDiscrete> weakRef = weak_from_this();
    monitorTimer.expires_after(std::chrono::milliseconds(pollTimeMs));
    monitorTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        std::shared_ptr<APISensorDiscrete> self = weakRef.lock();
        if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << debugMsgPrefix << "Failed to start sensor monitor\n";
            return;
        }
        if (self)
        {
            self->performRead();
        }
    });
}

void APISensorDiscrete::performRead(void)
{
    double reading;
    uint16_t newstate = 0;

    try
    {
        if (readFunction != nullptr)
        {
            reading = readFunction(name);

            struct stat file_info;
            bool debugShowReading =
                (0 == stat("/var/debugapisensor", &file_info));

            if (std::isnan(reading))
            {
                if (debugShowReading)
                {
                    std::cout << objType << ": " << name << ":" << readFuncName
                              << " returned: NaN" << std::endl;
                }
            }
            else
            {
                if (debugShowReading)
                {
                    std::cout << objType << ": " << name << ": " << readFuncName
                              << " returned: " << reading << std::endl;
                }
                newstate = static_cast<uint16_t>(
                    reading); // Get the lower 16 bits (newstate)

#ifdef DISCRETE_SENSOR_SEL_LOGGING
                std::vector<uint8_t> eventData(
                    3, 0xFF); // 3 bytes of event data initialized to 0xFF
                std::vector<std::string> logData;
                std::vector<std::pair<uint16_t, int>>
                    assertionsOccurred;   // assertion mask and bit position
                std::vector<std::pair<uint16_t, int>>
                    deassertionsOccurred; // deassertion mask and bit position
                std::string sPath = "/xyz/openbmc_project/sensors/" +
                                    dbusSensorType + "/" + name;
                std::string msg;
                std::string stateStr;
                std::map<std::string, std::string> addData;

                // For discrete snesors, the upper 16 bits of the reading
                // can be used to return sensor event data bytes 2 & 3.
                // One use case for this would be System Firmware Progress
                // sensors, where the read function can also return the post
                // progress code.
                uint32_t reading32 = static_cast<uint32_t>(reading);
                if (reading32 & 0xFF0000)
                {
                    eventData[1] =
                        static_cast<uint8_t>((reading32 & 0xFF0000) >> 16);
                }
                if (reading32 & 0xFF000000)
                {
                    eventData[2] =
                        static_cast<uint8_t>((reading32 & 0xFF000000) >> 24);
                }

                // Determine how many assertion events occurred
                for (auto i = 0; i < 16; i++)
                {
                    // Have we asserted any state bits?
                    if ((newstate & (1 << i)) && !(laststate & (1 << i)))
                    {
                        assertionsOccurred.push_back(
                            std::make_pair((1 << i), i));
                        if (debugShowReading)
                        {
                            std::cout
                                << objType << ": " << name
                                << ": asserted: " << (1 << i) << ", pos: " << i
                                << ", newstate: " << newstate
                                << ", laststate: " << laststate << std::endl;
                        }
                    }
                }

                // We detected assertion events
                if (assertionsOccurred.size() > 0)
                {
                    if (debugShowReading)
                    {
                        std::cout << objType << ": " << name
                                  << ": asserted states count: "
                                  << assertionsOccurred.size() << std::endl;
                    }

                    // For each assertion event, log the event
                    for (const auto& assertion : assertionsOccurred)
                    {
                        // Log the event
                        bool assert = true; // Assertion event
                        msg = "";
                        addData.clear();
                        eventData[0] = assertion.second;
                        std::string eventDataStr;
                        toHexStr(eventData, eventDataStr);
                        logData.clear();
                        logData.push_back(name);

                        stateStr =
                            (assertion.second < static_cast<int>(states.size()))
                                ? states[assertion.second]
                                : "n/a";
                        msg = stateStr + " ";
                        msg += std::string("assertion event.") +
                               std::string(" Reading=") +
                               std::to_string(newstate) +
                               std::string(" OldReading=") +
                               std::to_string(laststate) +
                               std::string(" Event=") + stateStr;
                        logData.push_back(msg);
                        logData.push_back(sPath);

                        addData["SENSOR_DATA"] = eventDataStr;
                        addData["SENSOR_PATH"] = sPath;
                        addData["EVENT_DIR"] = std::to_string(assert);
                        addData["GENERATOR_ID"] =
                            std::to_string(static_cast<uint16_t>(32));
                        addData["RECORD_TYPE"] =
                            std::to_string(static_cast<uint8_t>(2));
                        addData["SENSOR_TYPE"] =
                            std::to_string(static_cast<uint8_t>(typeCode));
                        addData["EVENT_TYPE"] = std::to_string(
                            static_cast<uint8_t>(eventReadingType));

                        if (debugShowReading)
                        {
                            std::cout
                                << objType << ": " << name << " " << msg
                                << std::string("SENSOR_DATA=") << eventDataStr
                                << std::string("SENSOR_PATH=") << sPath.c_str()
                                << std::string("EVENT_DIR=")
                                << std::to_string(assert)
                                << std::string("GENERATOR_ID")
                                << std::to_string(static_cast<uint16_t>(0x0020))
                                << std::string("RECORD_TYPE")
                                << std::to_string(static_cast<uint8_t>(0x02))
                                << std::string("SENSOR_TYPE")
                                << std::to_string(
                                       static_cast<uint8_t>(typeCode))
                                << std::string("EVENT_TYPE")
                                << std::to_string(
                                       static_cast<uint8_t>(eventReadingType))
                                << std::endl;
                        }
                        addSelEntry(dbusConnection, logData, eventData, assert,
                                    addData);

                        // If we have an InfoFunction and and the current sensor
                        // state matches the state needed for InfoFunction to be
                        // called...
                        if (infoFunction != nullptr && !infoFuncName.empty() &&
                            ((infoFuncCallState == stateStr) ||
                             (boost::to_lower_copy(infoFuncCallState) ==
                              "any")))
                        {
                            std::map<std::string, InfoVariantType> info;
                            int infoFuncError;
                            // Call InfoFunction to get custom Info
                            try
                            {
                                infoFuncError = infoFunction(name, &info);
                            }
                            catch (std::exception& e)
                            {
                                std::cerr << objType << ": " << name
                                          << ": InfoFunction failed! "
                                          << e.what() << std::endl;
                            }
                            // If InfoFunction call was successful
                            if (!infoFuncError)
                            {
                                if (!infoInterface) // If Info interface not yet
                                                    // created
                                {
                                    // First time through, create Info interface
                                    infoInterface = objServer.add_interface(
                                        objectPath,
                                        "xyz.openbmc_project.Sensor.Info");

                                    // For each key/value pair returned from
                                    // InfoFunction
                                    for (auto [key, val] : info)
                                    {
                                        if (debugShowReading)
                                        {
                                            std::cout
                                                << objType << ": " << name
                                                << ": Creating info property "
                                                << key << std::endl;
                                        }
                                        infoPropertyMap[key] =
                                            val; // Update info map
                                        // Register key/value pair as dbus
                                        // property/value
                                        registerInfoPropertyVariant(
                                            objType, name, key, val,
                                            infoInterface, infoPropertyMap);
                                    }
                                    // Initialize Info interface
                                    if (!infoInterface->initialize())
                                    {
                                        std::cerr
                                            << objType << ": " << name
                                            << ": error initializing info interface\n";
                                    }
                                }
                                else // Info interface already created
                                {
                                    // Update the dbus property values
                                    // with data from InfoFunction.
                                    // Note: Assumes InfoFunction returns the
                                    // same keys/values otherwise this logic
                                    // will fail!
                                    for (auto [key, val] : info)
                                    {
                                        infoPropertyMap[key] =
                                            val; // Update info property
                                    }
                                }
                            }
                            else
                            {
                                std::cerr
                                    << objType << ": " << name
                                    << ": InfoFunction failed!" << std::endl;
                            }
                        }
                    }
                }

                // Determine how many deassertion events occurred
                for (auto i = 0; i < 16; i++)
                {
                    // Have we deasserted any state bits?
                    if ((laststate & (1 << i)) && !(newstate & (1 << i)))
                    {
                        deassertionsOccurred.push_back(
                            std::make_pair((1 << i), i));
                        if (debugShowReading)
                        {
                            std::cout
                                << objType << ": " << name
                                << ": deasserted: " << (1 << i)
                                << ", pos: " << i << ", newstate: " << newstate
                                << ", laststate: " << laststate << std::endl;
                        }
                    }
                }

                // We detected deassertion events
                if (deassertionsOccurred.size() > 0)
                {
                    if (debugShowReading)
                    {
                        std::cout << objType << ": " << name
                                  << ": deasserted states count: "
                                  << deassertionsOccurred.size() << std::endl;
                    }

                    // Deassertion events are applicable for some discrete
                    // sensors but, not all. newstate==0 is normal for many
                    // sensor-specific discrete sensor types. For example:
                    //   - Chassis intrusion reports 0 when no intrustions have
                    //   occurred.
                    //   - Processor reports 0 when no errors have occurred.
                    // These types of sensors should log deassertin events
                    // Note: Ideally this should controlled via SDR
                    // assertion/deassertion mask
                    //       but that is not currently supported.
                    //
                    // Interate through the list of allowed deasserting sensor
                    // types
                    for (const auto& pair : deassertionSensorTypes)
                    {
                        // If sensor's event reading code and type code found in
                        // list (0xff = don't care)
                        if ((pair.first == 0xff ||
                             pair.first == eventReadingType) &&
                            (pair.second == 0xff || pair.second == typeCode))
                        {
                            // For each deassertion event, log the event
                            for (const auto& deassertion : deassertionsOccurred)
                            {
                                // Log the event
                                bool assert =
                                    false; // Deassertion event msg = "";
                                addData.clear();
                                eventData[0] = deassertion.second;
                                std::string eventDataStr;
                                toHexStr(eventData, eventDataStr);
                                logData.clear();
                                logData.push_back(name);

                                stateStr = (deassertion.second <
                                            static_cast<int>(states.size()))
                                               ? states[deassertion.second]
                                               : "n/a";
                                msg = stateStr + " ";

                                msg += std::string("deassertion event.") +
                                       std::string(" Reading=") +
                                       std::to_string(newstate) +
                                       std::string(" OldReading=") +
                                       std::to_string(laststate) +
                                       std::string(" Event=") + stateStr;
                                logData.push_back(msg);
                                logData.push_back(sPath);

                                addData["SENSOR_DATA"] = eventDataStr;
                                addData["SENSOR_PATH"] = sPath;
                                addData["EVENT_DIR"] = std::to_string(assert);
                                addData["GENERATOR_ID"] =
                                    std::to_string(static_cast<uint16_t>(32));
                                addData["RECORD_TYPE"] =
                                    std::to_string(static_cast<uint8_t>(2));
                                addData["SENSOR_TYPE"] = std::to_string(
                                    static_cast<uint8_t>(typeCode));
                                addData["EVENT_TYPE"] = std::to_string(
                                    static_cast<uint8_t>(eventReadingType));

                                if (debugShowReading)
                                {
                                    std::cout
                                        << objType << ": " << name << " " << msg
                                        << std::string("SENSOR_DATA=")
                                        << eventDataStr
                                        << std::string("SENSOR_PATH=")
                                        << sPath.c_str()
                                        << std::string("EVENT_DIR=")
                                        << std::to_string(assert)
                                        << std::string("GENERATOR_ID")
                                        << std::to_string(
                                               static_cast<uint16_t>(0x0020))
                                        << std::string("RECORD_TYPE")
                                        << std::to_string(
                                               static_cast<uint8_t>(0x02))
                                        << std::string("SENSOR_TYPE")
                                        << std::to_string(
                                               static_cast<uint8_t>(typeCode))
                                        << std::string("EVENT_TYPE")
                                        << std::to_string(static_cast<uint8_t>(
                                               eventReadingType))
                                        << std::endl;
                                }
                                addSelEntry(dbusConnection, logData, eventData,
                                            assert, addData);
                            }
                            break;
                        }
                    }
                }
#endif // #ifdef DISCRETE_SENSOR_SEL_LOGGING
            }
            laststate = newstate;
            updateState(sensorInterface, newstate);

            // if ProbeName provided, probe interface available,
            // and newstate not zero, update ProbeState property
            if (!probeName.empty() && probeInterface && newstate)
            {
                uint16_t bitpos = 1;
                for (const std::string& state : states)
                {
                    if (bitpos & newstate)
                    {
                        if (probeState != state)
                        {
                            probeState = state; // Update ProbeState property
                            std::cerr
                                << objType << ": " << probeName
                                << ": ProbeState set to " << state << "\n";
                        }
                    }
                    bitpos <<= 1;
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << debugMsgPrefix << "Exception in performRead() calling "
                  << readFuncName << "(): " << e.what() << std::endl;
    }
    startMonitor();
}

// Invoked by base class (Discrete.hpp) when
// sensor dbus "state" is updated externally
void APISensorDiscrete::externalSetTrigger(uint16_t newState)
{
    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << name << " dbus value externally set to "
                  << newState << "\n";
    }

    // External source modified sensor (dbus).  Check if we have
    // a WriteFunction. If so, call it to update hardware.
    if (writeFunction != nullptr)
    {
        if (0 != writeFunction(name, static_cast<double>(newState)))
        {
            std::cout << debugMsgPrefix << name << " WriteFunction failed!\n";
            // externalSetTrigger is called from within the dbus Set properties
            // command handler. Throwing an exception here should propagate
            // back to the dbus caller.
            throw SetSensorInternalFailure();
        }
    }
}

// Separate function from constructor, because of a gotcha: can't use the
// enable_shared_from_this() API until after the constructor has completed.
void APISensorDiscrete::initWriteHook(std::function<void()>&& writeHookIn)
{
    // Connect APISensorMain with APISensor if callback is provided
    if (writeHookIn != nullptr)
    {
        writeHook = std::move(writeHookIn);
    }

    // Connect APISensor with Sensor
    auto weakThis = weak_from_this();
    externalSetHook = [weakThis](uint16_t newState) {
        auto lockThis = weakThis.lock();
        if (lockThis)
        {
            lockThis->externalSetTrigger(newState);
            return;
        }
        if constexpr (debug)
        {
            std::cerr << debugMsgPrefix << "receive ignored, sensor gone\n";
        }
    };
}
