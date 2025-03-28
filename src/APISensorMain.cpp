#include "APISensor.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>
#include <dlfcn.h>
#include <type_traits>

// APISensor is a simple sensor interface where the readings are obtained from
// a library api (libapisensor.so). The libarary api function name is specifed in
// entity-manager JSON config file making it easy to create a simple sensor.
// The goal of APISensor is to simplify the process and reduce the time of
// creating sensors.
static constexpr bool debug = true;

static const char* libAPISensorFile = "/usr/lib/libapisensor.so.1.0.0";
static const std::string debugMsgPrefix = "APISensorMain: ";

void *libAPISensorHandle = nullptr;

template <typename T>
T getValueFromConfiguration(const std::string& configKey,
                            const std::string& interfacePath,
                            const SensorBaseConfigMap& baseConfigMap,
                            bool& found, bool& valid)
{
    T value;
    auto configFound = baseConfigMap.find(configKey);
    found = (configFound == baseConfigMap.end()) ? false : true;
    valid = false;

    // If key/value found
    if (found)
    {
        // If T is a std::string
        if constexpr (std::is_same_v<T, std::string>)
        {
            try
            {
                // Convert the config variant to a string
                value = std::visit(VariantToStringVisitor(), configFound->second);
                // If value is empty string, log error, value is invalid
                if (value.empty())
                {
                    if constexpr (debug)
                    {
                        std::cerr << debugMsgPrefix << configKey << " parameter cannot be parsed for "
                                  << interfacePath << "\n";
                    }
                }
                else
                {
                    valid = true; // Otherwise, consider value to be valid string
                }
            }
            catch (const std::exception& e)
            {
                if constexpr (debug)
                {
                    std::cerr << debugMsgPrefix << configKey << " parameter cannot be parsed for "
                              << interfacePath << "\n";
                }
                value = "";
            }
        }
        else if constexpr (std::is_same_v<T, double>) // T is a double
        {
            try
            {
                // Convert the config variant to a double
                value = std::visit(VariantToDoubleVisitor(), configFound->second);
                if (!std::isfinite(value)) // If value is not a valid double, log error
                {
                    if constexpr (debug)
                    {
                        std::cerr << debugMsgPrefix << configKey << " parameter cannot be parsed for "
                                  << interfacePath << "\n";
                    }
                }
                else
                {
                    valid = true; // Otherwise, consider value to be valid double
                }

            }
            catch (const std::exception& e)
            {
                if constexpr (debug)
                {
                    std::cerr << debugMsgPrefix << configKey << " parameter cannot be parsed for "
                              << interfacePath << "\n";
                }
                value = std::numeric_limits<double>::quiet_NaN();
            }
        }
        else if constexpr (std::is_same_v<T, std::vector<std::string>>) // T is a vector of strings
        {
            try
            {
                value = std::visit(VariantToStringArrayVisitor(), configFound->second);
                if (value.size() < 2) // Need at least 2 states for discrete sensors
                {
                    if constexpr (debug)
                    {
                        std::cerr << debugMsgPrefix << configKey << " parameter cannot be parsed for "
                                  << interfacePath << "\n";
                    }
                    value.clear(); // return empty vector
                }
                else
                {
                    valid = true; // Otherwise, consider value to be valid vector of strings
                }
            }
            catch (const std::invalid_argument& e)
            {
                if constexpr (debug)
                {
                    std::cerr << debugMsgPrefix << configKey << " parameter cannot be parsed for "
                                << interfacePath << "\n";
                }
                value.clear();
            }
        }
    }
    else // If key not found
    {
        if constexpr (debug)
        {
            std::cerr << debugMsgPrefix << configKey << " parameter not found for "
                      << interfacePath << "\n";
        }
    }
    // Caller must evaluate 'found' and 'valid' to determine success of call
    return value;
}

void createSensors(
    std::string& sensorType,
    sdbusplus::asio::object_server& objectServer,
    boost::asio::io_context& io,
    boost::container::flat_map<std::string, std::shared_ptr<APISensor>>&
        thresholdSensors,
    boost::container::flat_map<std::string, std::shared_ptr<APISensorDiscrete>>&
        discreteSensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << "Creating sensor objects\n";
    }

    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&sensorType, &objectServer, &io, &thresholdSensors, &discreteSensors, &dbusConnection, sensorsChanged]
        (const ManagedObjectType& sensorConfigurations) {
            bool firstScan = (sensorsChanged == nullptr);

            for (const std::pair<sdbusplus::message::object_path, SensorData>&
                     sensor : sensorConfigurations)
            {
                const std::string& interfacePath = sensor.first.str;
                const SensorData& sensorData = sensor.second;

                auto sensorBase =
                    sensorData.find(configInterfaceName(sensorType));
                if (sensorBase == sensorData.end())
                {
                    if constexpr (debug)
                    {
                        std::cerr << debugMsgPrefix << "Base configuration not found for "
                                  << interfacePath << "\n";
                    }
                    continue;
                }

                const SensorBaseConfiguration& baseConfiguration = *sensorBase;
                const SensorBaseConfigMap& baseConfigMap = baseConfiguration.second;
                bool found = false, valid = false;

                // First check for "Units" or "State" properties
                // APISensor uses these properties to determine
                // threshold vs discrete sensor.
                bool unitsFound = false, unitsValid = false;
                std::string sensorUnits =
                        getValueFromConfiguration<std::string>(
                                "Units", interfacePath, baseConfigMap,
                                unitsFound, unitsValid);

                bool stateFound = false, stateValid = false;
                std::vector<std::string> sensorStates =
                        getValueFromConfiguration<std::vector<std::string>>(
                                "State", interfacePath, baseConfigMap,
                                stateFound, stateValid);

                // Discrete sensors must have valid State parameter
                // and no Units parameter
                bool sensorIsDiscrete = (stateValid && !unitsFound) ? true : false;

                // Threshold sensors must have valid Units parameter
                // and no State parameter
                bool sensorIsThreshold = (unitsValid && !stateFound) ? true : false;

                // If not discrete and not threshold sensor, skip it
                if (!sensorIsDiscrete && !sensorIsThreshold)
                {
                    if constexpr (debug)
                    {
                        std::cerr << debugMsgPrefix << "Unknown sensor type for "
                                  << interfacePath << "\n";
                    }
                    continue;
                }

                // Name is mandatory string parameter
                std::string sensorName =
                    getValueFromConfiguration<std::string>(
                            "Name", interfacePath, baseConfigMap, found, valid);
                if (!found || (found && !valid)) { continue; }

                // ReadFunction is mandatory string parameter
                std::string sensorReadFuncName =
                    getValueFromConfiguration<std::string>(
                            "ReadFunction", interfacePath, baseConfigMap, found, valid);
                if (!found || (found && !valid)) { continue; }

                // InitFunctionName is optional string parameter
                std::string sensorInitFuncName =
                    getValueFromConfiguration<std::string>(
                            "InitFunction", interfacePath, baseConfigMap, found, valid);

                // WriteFunction is optional string parameter
                std::string sensorWriteFuncName =
                    getValueFromConfiguration<std::string>(
                        "WriteFunction", interfacePath, baseConfigMap, found, valid);

                // PollRate is optional numeric parameter for threshold sensors
                unsigned long sensorPollTimeMs = 5000UL; // default to 5 seconds
                double pollRate =
                    getValueFromConfiguration<double>(
                        "PollRate", interfacePath, baseConfigMap, found, valid);
                if (found && valid)
                {
                    sensorPollTimeMs = static_cast<unsigned long>(pollRate * 1000.0);
                    // Set minimum poll rate to 1 second
                    if (sensorPollTimeMs < 1000UL)
                    {
                        if constexpr (debug)
                        {
                            std::cerr << debugMsgPrefix << sensorName << " PollRate="
                                      << sensorPollTimeMs << "ms too low, setting 1sec\n";
                        }
                        sensorPollTimeMs = 1000UL;
                    }
                }

                // PowerState is optional parameter
                PowerState readState = getPowerState(baseConfigMap);

                // EntityId, EnityInstance, SensorTypeCode, and EventReadingType
                // are optional numeric parameters used by the IPMI to override
                // their corresponding SDR fields during SDR creation.
                // APISensor currently does nothing with these values. We are just
                // getting their values incase we need them in the future.
                // These values are for IPMI only. Therefore, if IPMI is not used,
                // these values need not be defined in entity-manager.
                double sensorEntityId =
                    getValueFromConfiguration<double>(
                        "EntityId", interfacePath, baseConfigMap, found, valid);

                double sensorEntityInstance =
                    getValueFromConfiguration<double>(
                        "EntityInstance", interfacePath, baseConfigMap, found, valid);

                double sensorTypeCode =
                    getValueFromConfiguration<double>(
                        "SensorTypeCode", interfacePath, baseConfigMap, found, valid);

                double sensorEventReadingType =
                    getValueFromConfiguration<double>(
                        "EventReadingType", interfacePath, baseConfigMap, found, valid);

                // TypePath is optional string parameter that
                // allows us to override the sensor type path.
                // There currently is no use case for this parameter.
                std::string sensorTypePathOverride =
                    getValueFromConfiguration<std::string>(
                        "TypePath", interfacePath, baseConfigMap, found, valid);

                // Threshold sensor parameters
                double sensorScale = 1; // optional: default scale to 1
                double maxValue;        // mandatory
                double minValue;        // mandatory
                std::vector<thresholds::Threshold> sensorThresholds;
                if (!sensorIsDiscrete)
                {
                    // MaxValue is mandatory numeric parameter for threshold sensors
                    maxValue =
                        getValueFromConfiguration<double>(
                            "MaxValue", interfacePath, baseConfigMap, found, valid);
                    if (!found || (found && !valid)) { continue; }


                    // MinValue is mandatory numeric parameter for threshold sensors
                    minValue =
                        getValueFromConfiguration<double>(
                            "MinValue", interfacePath, baseConfigMap, found, valid);
                    if (!found || (found && !valid)) { continue; }

                    // ScaleFactor is optional numeric parameter for threshold sensors
                    sensorScale =
                        getValueFromConfiguration<double>(
                            "ScaleFactor", interfacePath, baseConfigMap, found, valid);
                    if (found && !valid) { sensorScale = 1; }

                    // Parse thresholds for threshold sensors only
                    if (!parseThresholdsFromConfig(sensorData, sensorThresholds))
                    {
                        if constexpr (debug)
                        {
                            std::cerr << debugMsgPrefix << sensorName << " error populating thresholds\n";
                        }
                    }
                }

                if (sensorIsDiscrete)
                {
                    // on rescans, only update sensors we were signaled by
                    auto findSensor = discreteSensors.find(sensorName);
                    if (!firstScan && (findSensor != discreteSensors.end()))
                    {
                        std::string suffixName = "/";
                        suffixName += findSensor->second->name;
                        bool found = false;
                        for (auto it = sensorsChanged->begin();
                            it != sensorsChanged->end(); it++)
                        {
                            std::string suffixIt = "/";
                            suffixIt += *it;
                            if (suffixIt.ends_with(suffixName))
                            {
                                sensorsChanged->erase(it);
                                findSensor->second = nullptr;
                                found = true;
                                if constexpr (debug)
                                {
                                    if constexpr (debug)
                                    {
                                        std::cout << debugMsgPrefix << sensorName << " change found\n";
                                    }
                                }
                                break;
                            }
                        }
                        if (!found)
                        {
                            continue;
                        }
                    }

                    auto& sensorEntry = discreteSensors[sensorName];
                    sensorEntry = nullptr;

                    sensorEntry = std::make_shared<APISensorDiscrete>(
                            sensorType, objectServer, io, dbusConnection, sensorName,
                            interfacePath, readState, sensorEventReadingType,
                            sensorTypeCode, sensorEntityId, sensorEntityInstance,
                            sensorTypePathOverride, sensorStates, sensorInitFuncName, sensorReadFuncName,
                            sensorWriteFuncName, sensorPollTimeMs, libAPISensorHandle);
                    // If sensor library is loaded and we have a valid pointer
                    // to the ReadFunction, then start the sensor monitor loop.
                    if (libAPISensorHandle && (sensorEntry->readFunction != nullptr))
                    {
                        sensorEntry->startMonitor(); // Start sensor monitor wait timer
                    }

                    // Call to setup externalSetHook within sensor class.
                    sensorEntry->initWriteHook(nullptr); // Not using callback here
                }
                else
                {
                    // on rescans, only update sensors we were signaled by
                    auto findSensor = thresholdSensors.find(sensorName);
                    if (!firstScan && (findSensor != thresholdSensors.end()))
                    {
                        std::string suffixName = "/";
                        suffixName += findSensor->second->name;
                        bool found = false;
                        for (auto it = sensorsChanged->begin();
                            it != sensorsChanged->end(); it++)
                        {
                            std::string suffixIt = "/";
                            suffixIt += *it;
                            if (suffixIt.ends_with(suffixName))
                            {
                                sensorsChanged->erase(it);
                                findSensor->second = nullptr;
                                found = true;
                                if constexpr (debug)
                                {
                                    if constexpr (debug)
                                    {
                                        std::cout << debugMsgPrefix << sensorName << " change found\n";
                                    }
                                }
                                break;
                            }
                        }
                        if (!found)
                        {
                            continue;
                        }
                    }

                    auto& sensorEntry = thresholdSensors[sensorName];
                    sensorEntry = nullptr;

                    sensorEntry = std::make_shared<APISensor>(
                            sensorType, objectServer, io, dbusConnection, sensorName,
                            sensorUnits, std::move(sensorThresholds), interfacePath,
                            maxValue, minValue, readState, sensorScale, sensorEventReadingType,
                            sensorTypeCode, sensorEntityId, sensorEntityInstance,
                            sensorTypePathOverride, sensorInitFuncName, sensorReadFuncName,
                            sensorWriteFuncName, sensorPollTimeMs, libAPISensorHandle);

                    // If sensor library is loaded and we have a valid pointer
                    // to the ReadFunction, then start the sensor monitor loop.
                    if (libAPISensorHandle && (sensorEntry->readFunction != nullptr))
                    {
                        sensorEntry->startMonitor(); // Start sensor monitor wait timer
                    }

                    // Call to setup externalSetHook within sensor class.
                    sensorEntry->initWriteHook(nullptr); // Not using callback here

                }
            }
        });

    getter->getConfiguration(std::vector<std::string>{sensorType});
}

int main(int argc, char* argv[])
{
    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << " service starting up\n";
    }

    // Load libapisensor.so library
    libAPISensorHandle = dlopen(libAPISensorFile, RTLD_LAZY);
    if (!libAPISensorHandle)
    {
        std::cerr << debugMsgPrefix << "Failed to load " << libAPISensorFile << " - Sensor reading/writing will be unavailable\n";
    }

    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);

    objectServer.add_manager("/xyz/openbmc_project/sensors");
    std::string sensorType = "APISensor";
    std::string serviceName = "xyz.openbmc_project.APISensor";

    // Add capability to spread sensor monitoring load across multiple
    // APISensor reactors (e.g. APISensor, APISensor2, APISensor3, etc.)
    //   xyz.openbmc_project.APISensor    (JSON: "Type": "APISensor")
    //   xyz.openbmc_project.APISensor2   (JSON: "Type": "APISensor2")
    //   xyz.openbmc_project.APISensor3   (JSON: "Type": "APISensor3")
    //   xyz.openbmc_project.APISensor4   (JSON: "Type": "APISensor4")
    if (argc > 1)
    {
        serviceName += std::string(argv[1]);// "xyz.openbmc_project.APISensor" + argv[1]
        sensorType += std::string(argv[1]); // "APISensor" + argv[1]
    }

    systemBus->request_name(serviceName.c_str());

    boost::container::flat_map<std::string, std::shared_ptr<APISensor>>
        thresholdSensors;
    boost::container::flat_map<std::string, std::shared_ptr<APISensorDiscrete>>
        discreteSensors;

    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    boost::asio::post(io, [&sensorType, &objectServer, &io, &thresholdSensors, &discreteSensors, &systemBus]() {
        createSensors(sensorType, objectServer, io, thresholdSensors, discreteSensors, systemBus, nullptr);
    });

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&sensorType, &objectServer, &io, &thresholdSensors, &discreteSensors, &systemBus, &sensorsChanged, &filterTimer]
        (sdbusplus::message_t& message) mutable {
            if (message.is_method_error())
            {
                std::cerr << debugMsgPrefix << "callback method error\n";
                return;
            }

            const auto* messagePath = message.get_path();
            sensorsChanged->insert(messagePath);
            if constexpr (debug)
            {
                std::cout << debugMsgPrefix << "Change event received: "
                          << messagePath << "\n";
            }

            // this implicitly cancels the timer
            filterTimer.expires_after(std::chrono::seconds(1));

            filterTimer.async_wait(
                [&sensorType, &objectServer, &io, &thresholdSensors, &discreteSensors, &systemBus, &sensorsChanged]
                (const boost::system::error_code& ec) mutable {
                    if (ec != boost::system::errc::success)
                    {
                        if (ec != boost::asio::error::operation_aborted)
                        {
                            std::cerr << debugMsgPrefix << "callback error: " << ec.message() << "\n";
                        }
                        return;
                    }

                    createSensors(sensorType, objectServer, io, thresholdSensors, discreteSensors, systemBus,
                                  sensorsChanged);
                });
        };

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(
            *systemBus, std::to_array<const char*>({sensorType.c_str()}), eventHandler);

    if constexpr (debug)
    {
        std::cout << debugMsgPrefix << "service entering main loop\n";
    }

    io.run();
}
