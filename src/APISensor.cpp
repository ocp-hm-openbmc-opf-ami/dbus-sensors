#include "APISensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <boost/algorithm/string.hpp>
#include <sys/stat.h>

#include <chrono>
#include <cstddef>
#include <functional>
#include <iostream>
#include <sstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <dlfcn.h>
#include <cmath>

#include <unistd.h>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#define DISCRETE_SENSOR_SEL_LOGGING // TODO: move to meson.options

static constexpr bool debug = true;
static const std::string debugMsgPrefix = "APISensor: ";

APISensor::APISensor(
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
    [[maybe_unused]] const PowerState& powerState,
    double sensorScale,
    [[maybe_unused]] const double sensorEventReadingType,
    [[maybe_unused]] const double sensorTypeCode,
    [[maybe_unused]] const double sensorEntityId,
    [[maybe_unused]] const double sensorEntityInstance,
    const std::string& sensorTypePathOverride,
    const std::string& sensorInitFuncName,
    const std::string& sensorReadFuncName,
    const std::string& sensorWriteFuncName,
    unsigned long sensorPollTimeMs,
    void *libHandle) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectType, true, true, maxReading, minReading, conn, powerState),
    objType(objectType),
    objServer(objectServer),
    monitorTimer(io),
    scale(sensorScale),
    eventReadingType(sensorEventReadingType),
    typeCode(sensorTypeCode),
    entityId(sensorEntityId),
    entityInstance(sensorEntityInstance),
    typePathOverride(sensorTypePathOverride),
    initFuncName(sensorInitFuncName),
    readFuncName(sensorReadFuncName),
    writeFuncName(sensorWriteFuncName),
    pollTimeMs(sensorPollTimeMs),
    libAPISensorHandle(libHandle)
{
    std::string dbusPath = getSensorPathFromUnits(sensorUnits, entityId);
    if (dbusPath.empty())
    {
        throw std::runtime_error("Units not in allow list");
    }

    // If sensor has an InitFunction, get function pointer and call it
    if (!initFuncName.empty())
    {
        // Get pointer to InitFunction
        dlerror(); // First, clear any existing dlerror
        initFunction = (initFunction_T)dlsym(libAPISensorHandle, initFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error || (initFunction == nullptr))
        {
            std::cerr << debugMsgPrefix << name << " Cannot find function symbol: "
                      << initFuncName << " - " << dlsym_error << "\n";
            // InitFunction is optional, continue on without it
        }
        else
        {
            // Call the init function
            int initFuncErrorCode= initFunction();
            if (initFuncErrorCode < 0)
            {
                std::cerr << debugMsgPrefix << name << " Error calling init function "
                          <<  initFuncName << ": " << initFuncErrorCode << "\n";
                // InitFunction failed but, InitFunction is optional, so conitue on
            }
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
        readFunction = (readFunction_T)dlsym(libAPISensorHandle, readFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error || (readFunction == nullptr))
        {
            std::cerr << debugMsgPrefix << name << " Cannot find function symbol: "
                      << readFuncName << " - " << dlsym_error << "\n";
            // ReadFunction not found, ReadFunction is mandatory to obtain sensor readings.
            // Without it, the sensor will always report NaN as the value
        }
    }

    // WriteFunction is optional
    if (!writeFuncName.empty())
    {
        // Get pointer to WriteFunction
        dlerror(); // First, clear any existing dlerror
        writeFunction = (writeFunction_T)dlsym(libAPISensorHandle, writeFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error || (writeFunction == nullptr))
        {
            std::cerr << debugMsgPrefix << name << " Cannot find function symbol: "
                        << writeFuncName << " - " << dlsym_error << "\n";
            // WriteFunction is optional, continue on without it
        }
    }

    std::string objectPath = "/xyz/openbmc_project/sensors/";
    objectPath += typePathOverride.empty() ? dbusPath : typePathOverride;
    objectPath += '/';
    objectPath += sensorName;

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
        std::cout << debugMsgPrefix << name
                  << " constructed APISensor path " << configurationPath
                  << ", readfunc " << readFuncName
                  << std::endl;
    }
}

std::string APISensor::getSensorPathFromUnits(const std::string& sensorUnits,
                                              [[maybe_unused]] const double& entityId)
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
    externalSetHook = [weakThis]() {
        auto lockThis = weakThis.lock();
        if (lockThis)
        {
            lockThis->externalSetTrigger();
            return;
        }
        if constexpr (debug)
        {
            std::cerr << debugMsgPrefix << "receive ignored, sensor gone\n";
        }
    };
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

void APISensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}

// Invoked by base class (sensor.hpp) when
// sensor dbus value is updated externally
void APISensor::externalSetTrigger()
{
    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << name << " dbus value externally set to " << value << "\n";
    }

    // External source modified sensor (dbus).  Check if we have
    // a WriteFunction. If so, call it to update hardware.
    if (writeFunction != nullptr)
    {
        if (0 != writeFunction(value))
        {
            std::cout << debugMsgPrefix << name << " WriteFunction failed!\n";
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
            reading = readFunction();

            struct stat file_info;
            bool debugShowReading = (0 == stat("/var/debugapisensor", &file_info));

            if (std::isnan(reading))
            {
                if (debugShowReading)
                {
                    std::cout << debugMsgPrefix << readFuncName << " returned: NaN" << std::endl;
                }
                updateValue(std::numeric_limits<double>::quiet_NaN());
            }
            else
            {
#if 0 // Option to reduce reading to 2 decimal places
                double multiplier = std::pow(10, 2);
                reading = std::round(reading * multiplier) / multiplier;
#endif
                double scaledReading = reading * scale; // Multiple reading by ScaleFactor
                if (debugShowReading)
                {
                    std::cout << debugMsgPrefix << readFuncName << " returned: " << reading
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
            std::cerr << debugMsgPrefix << "Failed to start sensor monitor\n";
            return;
        }
        if (self)
        {
            self->performRead();
        }
    });
}




APISensorDiscrete::APISensorDiscrete(
    const std::string& objectType,
    sdbusplus::asio::object_server& objectServer,
    boost::asio::io_context& io,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& sensorName,
    const std::string& sensorConfiguration,
    [[maybe_unused]] const PowerState& powerState,
    [[maybe_unused]] const double sensorEventReadingType,
    [[maybe_unused]] const double sensorTypeCode,
    [[maybe_unused]] const double sensorEntityId,
    [[maybe_unused]] const double sensorEntityInstance,
    const std::string& sensorTypePathOverride,
    const std::vector<std::string>& sensorStates,
    const std::string& sensorInitFuncName,
    const std::string& sensorReadFuncName,
    const std::string& sensorWriteFuncName,
    unsigned long sensorPollTimeMs,
    void *libHandle) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objType(objectType),
    objServer(objectServer),
    monitorTimer(io),
    eventReadingType(sensorEventReadingType),
    typeCode(sensorTypeCode),
    entityId(sensorEntityId),
    entityInstance(sensorEntityInstance),
    typePathOverride(sensorTypePathOverride),
    states(sensorStates),
    initFuncName(sensorInitFuncName),
    readFuncName(sensorReadFuncName),
    writeFuncName(sensorWriteFuncName),
    pollTimeMs(sensorPollTimeMs),
    libAPISensorHandle(libHandle)
{
    // If ReadFunction is missing, sensor will not be operational
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
        readFunction = (readFunction_T)dlsym(libAPISensorHandle, readFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error)
        {
            std::cerr << debugMsgPrefix << name << " Cannot find function symbol: "
                      << readFuncName << " - " << dlsym_error << "\n";
            // Do not close the handle (it's needed for other sensors)
        }
    }

    // WriteFunction is optional
    if (!writeFuncName.empty())
    {
        // Get pointer to WriteFunction
        dlerror(); // Clear any existing error
        writeFunction = (writeFunction_T)dlsym(libAPISensorHandle, writeFuncName.c_str());
        const char* dlsym_error = dlerror();
        if (dlsym_error)
        {
            std::cerr << debugMsgPrefix << name << " Cannot find function symbol: "
                        << writeFuncName << " - " << dlsym_error << "\n";
            // Do not close the handle (it's needed for other sensors)
        }
    }


    // Setup the dbus path for the sensor
    std::string sensorType = "discrete";
    std::string objectPath = "/xyz/openbmc_project/sensors/";
    objectPath += typePathOverride.empty() ? sensorType : typePathOverride;
    objectPath += '/';
    objectPath += sensorName;

    sensorInterface = objectServer.add_interface(
        objectPath, "xyz.openbmc_project.Sensor.State");

    association = objectServer.add_interface(
        objectPath, association::interface);

    setInitialProperties();

    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << name
                  << " constructed: LibAPISensorDiscrete path " << configurationPath
                  << ", type " << objectType
                  << ", readfunc " << readFuncName
                  << std::endl;
    }
}

APISensorDiscrete::~APISensorDiscrete()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
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
            reading = readFunction();

            struct stat file_info;
            bool debugShowReading = (0 == stat("/var/debugapisensor", &file_info));

            if (std::isnan(reading))
            {
                if (debugShowReading)
                {
                    std::cout << objType << ": " << name << ":" << readFuncName << " returned: NaN" << std::endl;
                }
            }
            else
            {
                if (debugShowReading)
                {
                    std::cout << objType << ": " << name << ": " << readFuncName << " returned: " << reading << std::endl;
                }
                newstate = static_cast<uint16_t>(reading); // Get the lower 16 bits (newstate)
#ifdef DISCRETE_SENSOR_SEL_LOGGING

                std::vector<uint8_t> eventData(3, 0xFF); // 3 bytes of event data initialized to 0xFF
                std::vector<std::string> logData;
                std::vector<std::pair<uint16_t, int>> assertionsOccurred;   // assertion mask and bit position
                std::vector<std::pair<uint16_t, int>> deassertionsOccurred; // deassertion mask and bit position
                std::string sPath = "/xyz/openbmc_project/sensors/" + objType + "/" + name;
                std::string msg;
                std::map<std::string, std::string> addData;

                // For discrete snesors, the upper 16 bits of the reading
                // can be used to return sensor event data bytes 2 & 3.
                // One use case for this would be System Firmware Progress
                // sensors, where the read function can also return the post
                // progress code.
                uint32_t reading32 = static_cast<uint32_t>(reading);
                if (reading32 & 0xFF0000)
                {
                    eventData[1] = static_cast<uint8_t>((reading32 & 0xFF0000) >> 16);
                }
                if (reading32 & 0xFF000000)
                {
                    eventData[2] = static_cast<uint8_t>((reading32 & 0xFF000000) >> 24);
                }

                // Determine how many assertion events occurred
                for (auto i = 0; i < 16; i++)
                {
                    // Have we asserted any state bits?
                    if ((newstate & (1 << i)) && !(laststate & (1 << i)))
                    {
                        assertionsOccurred.push_back(std::make_pair((1 << i), i));
                        if (debugShowReading)
                        {
                            std::cout << objType << ": " << name << ": asserted: "
                                      << (1 << i) << ", pos: " << i << ", newstate: "
                                      << newstate << ", laststate: " << laststate << std::endl;
                        }
                    }
                }

                // We detected assertion events
                if (assertionsOccurred.size() > 0)
                {
                    if (debugShowReading)
                    {
                        std::cout << objType << ": " << name << ": asserted states count: "
                                  << assertionsOccurred.size() << std::endl;
                    }

                    // For each assertion event, log the event
                    for (const auto& assertion : assertionsOccurred)
                    {
                        // Log the event
                        bool assert = true;  // Assertion event
                        msg = "";
                        addData.clear();
                        eventData[0] = assertion.second;
                        std::string eventDataStr;
                        toHexStr(eventData, eventDataStr);
                        logData.clear();
                        logData.push_back(name);
                        if (states.size() > 0)
                        {
                            // If availeble, prepend the state name to the log message
                            msg = states[assertion.second] + " ";
                        }
                        msg += std::string("assertion event.") +
                            std::string(" Reading=") + std::to_string(newstate) +
                            std::string(" OldReading=") + std::to_string(laststate) +
                            std::string(" Offset=") + std::to_string(assertion.second);
                        logData.push_back(msg);
                        logData.push_back(sPath);

                        addData["SENSOR_DATA"] = eventDataStr;
                        addData["SENSOR_PATH"] = sPath;
                        addData["EVENT_DIR"] = std::to_string(assert);
                        addData["GENERATOR_ID"] = std::to_string(static_cast<uint16_t>(32));
                        addData["RECORD_TYPE"] = std::to_string(static_cast<uint8_t>(2));
                        addData["SENSOR_TYPE"] = std::to_string(typeCode);
                        addData["EVENT_TYPE"] = std::to_string(eventReadingType);

                        if (debugShowReading)
                        {
                            std::cout << objType << ": " << name << " " << msg
                                      << std::string("SENSOR_DATA=") << eventDataStr
                                      << std::string("SENSOR_PATH=") << sPath.c_str()
                                      << std::string("EVENT_DIR=") << std::to_string(assert)
                                      << std::string("GENERATOR_ID") << std::to_string(static_cast<uint16_t>(0x0020))
                                      << std::string("RECORD_TYPE") << std::to_string(static_cast<uint8_t>(0x02))
                                      << std::string("SENSOR_TYPE") << std::to_string(typeCode)
                                      << std::string("EVENT_TYPE") << std::to_string(eventReadingType)
                                      << std::endl;
                        }
                        addSelEntry(dbusConnection, logData, eventData, assert, addData);
                    }
                }

                // Determine how many deassertion events occurred
                for (auto i = 0; i < 16; i++)
                {
                    // Have we deasserted any state bits?
                    if ((laststate & (1 << i)) && !(newstate & (1 << i)))
                    {
                        deassertionsOccurred.push_back(std::make_pair((1 << i), i));
                        if (debugShowReading)
                        {
                            std::cout << objType << ": " << name << ": deasserted: "
                                      << (1 << i) << ", pos: " << i << ", newstate: "
                                      << newstate << ", laststate: " << laststate << std::endl;
                        }

                    }
                }

                // We detected deassertion events
                if (deassertionsOccurred.size() > 0)
                {
                    if (debugShowReading)
                    {
                        std::cout << objType << ": " << name << ": deasserted states count: "
                                  << deassertionsOccurred.size() << std::endl;
                    }

                    // Deassertion events are applicable for some discrete sensors but, not all.
                    // newstate==0 is normal for many sensor-specific discrete sensor types.
                    // For example:
                    //   - Chassis intrusion reports 0 when no intrustions have occurred.
                    //   - Processor reports 0 when no errors have occurred.
                    // These types of sensors should log deassertin events
                    // Note: Ideally this should controlled via SDR assertion/deassertion mask
                    //       but that is not currently supported.
                    //
                    // Interate through the list of allowed deasserting sensor types
                    for (const auto& pair : deassertionSensorTypes)
                    {
                        // If sensor's event reading code and type code found in list (0xff = don't care)
                        if ((pair.first == 0xff || pair.first == eventReadingType) &&
                            (pair.second == 0xff || pair.second == typeCode))
                        {
                            // For each deassertion event, log the event
                            for (const auto& deassertion : deassertionsOccurred)
                            {
                                // Log the event
                                bool assert = false; // Deassertion event                                msg = "";
                                addData.clear();
                                eventData[0] = deassertion.second;
                                std::string eventDataStr;
                                toHexStr(eventData, eventDataStr);
                                logData.clear();
                                logData.push_back(name);
                                if (states.size() > 0) // If state string names are available
                                {
                                    // Prepend the state name to the log message
                                    msg = states[deassertion.second] + " ";
                                }
                                msg += std::string("deassertion event.") +
                                    std::string(" Reading=") + std::to_string(newstate) +
                                    std::string(" OldReading=") + std::to_string(laststate) +
                                    std::string(" Offset=") + std::to_string(deassertion.second);
                                logData.push_back(msg);
                                logData.push_back(sPath);

                                addData["SENSOR_DATA"] = eventDataStr;
                                addData["SENSOR_PATH"] = sPath;
                                addData["EVENT_DIR"] = std::to_string(assert);
                                addData["GENERATOR_ID"] = std::to_string(static_cast<uint16_t>(32));
                                addData["RECORD_TYPE"] = std::to_string(static_cast<uint8_t>(2));
                                addData["SENSOR_TYPE"] = std::to_string(typeCode);
                                addData["EVENT_TYPE"] = std::to_string(eventReadingType);

                                if (debugShowReading)
                                {
                                    std::cout << objType << ": " << name << " " << msg
                                    << std::string("SENSOR_DATA=") << eventDataStr
                                    << std::string("SENSOR_PATH=") << sPath.c_str()
                                    << std::string("EVENT_DIR=") << std::to_string(assert)
                                    << std::string("GENERATOR_ID") << std::to_string(static_cast<uint16_t>(0x0020))
                                    << std::string("RECORD_TYPE") << std::to_string(static_cast<uint8_t>(0x02))
                                    << std::string("SENSOR_TYPE") << std::to_string(typeCode)
                                    << std::string("EVENT_TYPE") << std::to_string(eventReadingType)
                                    << std::endl;
                                }
                                addSelEntry(dbusConnection, logData, eventData, assert, addData);
                            }
                            break;
                        }
                    }
                }
#endif // #ifdef DISCRETE_SENSOR_SEL_LOGGING
            }
            laststate = newstate;
            updateState(sensorInterface, newstate);
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
// sensor dbus value is updated externally
void APISensorDiscrete::externalSetTrigger()
{
    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << name << " dbus value externally set to " << state << "\n";
    }

    // External source modified sensor (dbus).  Check if we have
    // a WriteFunction. If so, call it to update hardware.
    if (writeFunction != nullptr)
    {
        if (0 != writeFunction(static_cast<double>(state)))
        {
            std::cout << debugMsgPrefix << name << " WriteFunction failed!\n";
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
    externalSetHook = [weakThis]() {
        auto lockThis = weakThis.lock();
        if (lockThis)
        {
            lockThis->externalSetTrigger();
            return;
        }
        if constexpr (debug)
        {
            std::cerr << debugMsgPrefix << "receive ignored, sensor gone\n";
        }
    };
}
