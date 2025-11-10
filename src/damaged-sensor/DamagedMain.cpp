
#include "DamagedSensor.hpp"
#include "DeviceMgmt.hpp"
#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"

#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/message/native_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <ios>
#include <iostream>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

namespace fs = std::filesystem;

static const I2CDeviceTypeMap sensorTypes{
    {"ADM1021", I2CDeviceType{"adm1021", true}},
    {"DPS310", I2CDeviceType{"dps310", false}},
    {"EMC1412", I2CDeviceType{"emc1412", true}},
    {"EMC1413", I2CDeviceType{"emc1413", true}},
    {"EMC1414", I2CDeviceType{"emc1414", true}},
    {"HDC1080", I2CDeviceType{"hdc1080", false}},
    {"JC42", I2CDeviceType{"jc42", true}},
    {"LM75A", I2CDeviceType{"lm75a", true}},
    {"LM95234", I2CDeviceType{"lm95234", true}},
    {"MAX31725", I2CDeviceType{"max31725", true}},
    {"MAX31730", I2CDeviceType{"max31730", true}},
    {"MAX6581", I2CDeviceType{"max6581", true}},
    {"MAX6654", I2CDeviceType{"max6654", true}},
    {"MAX6639", I2CDeviceType{"max6639", true}},
    {"MCP9600", I2CDeviceType{"mcp9600", false}},
    {"NCT6779", I2CDeviceType{"nct6779", true}},
    {"NCT7802", I2CDeviceType{"nct7802", true}},
    {"PT5161L", I2CDeviceType{"pt5161l", true}},
    {"SBTSI", I2CDeviceType{"sbtsi", true}},
    {"SI7020", I2CDeviceType{"si7020", false}},
    {"TMP100", I2CDeviceType{"tmp100", true}},
    {"TMP112", I2CDeviceType{"tmp112", true}},
    {"TMP175", I2CDeviceType{"tmp175", true}},
    {"TMP421", I2CDeviceType{"tmp421", true}},
    {"TMP441", I2CDeviceType{"tmp441", true}},
    {"TMP461", I2CDeviceType{"tmp461", true}},
    {"TMP464", I2CDeviceType{"tmp464", true}},
    {"TMP75", I2CDeviceType{"tmp75", true}},
    {"W83773G", I2CDeviceType{"w83773g", true}},
};

struct SensorConfigKey
{
    uint64_t bus;
    uint64_t addr;
    bool operator<(const SensorConfigKey& other) const
    {
        if (bus != other.bus)
        {
            return bus < other.bus;
        }
        return addr < other.addr;
    }
};

struct SensorConfig
{
    std::string sensorPath;
    SensorData sensorData;
    std::string interface;
    SensorBaseConfigMap config;
    std::vector<std::string> name;
};

using SensorConfigMap =
    boost::container::flat_map<SensorConfigKey, SensorConfig>;

static SensorConfigMap buildSensorConfigMap(
    const ManagedObjectType& sensorConfigs)
{
    SensorConfigMap configMap;
    for (const auto& [path, cfgData] : sensorConfigs)
    {
        for (const auto& [intf, cfg] : cfgData)
        {
            auto busCfg = cfg.find("Bus");
            auto addrCfg = cfg.find("Address");
            if ((busCfg == cfg.end()) || (addrCfg == cfg.end()))
            {
                continue;
            }

            if ((std::get_if<uint64_t>(&busCfg->second) == nullptr) ||
                (std::get_if<uint64_t>(&addrCfg->second) == nullptr))
            {
                std::cerr << path.str << " Bus or Address invalid\n";
                continue;
            }

            std::vector<std::string> hwmonNames;
            auto nameCfg = cfg.find("Name");
            if (nameCfg != cfg.end())
            {
                hwmonNames.push_back(std::get<std::string>(nameCfg->second));
                size_t i = 1;
                while (true)
                {
                    auto sensorNameCfg = cfg.find("Name" + std::to_string(i));
                    if (sensorNameCfg == cfg.end())
                    {
                        break;
                    }
                    hwmonNames.push_back(
                        std::get<std::string>(sensorNameCfg->second));
                    i++;
                }
            }

            SensorConfigKey key = {std::get<uint64_t>(busCfg->second),
                                   std::get<uint64_t>(addrCfg->second)};
            SensorConfig val = {path.str, cfgData, intf, cfg, hwmonNames};

            auto [it, inserted] = configMap.emplace(key, std::move(val));
            if (!inserted)
            {
                std::cerr << path.str << ": ignoring duplicate entry for {"
                          << key.bus << ", 0x" << std::hex << key.addr
                          << std::dec << "}\n";
            }
        }
    }
    return configMap;
}

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<DamagedSensor>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    [[maybe_unused]] bool activateOnly)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &objectServer, &sensors, &dbusConnection, sensorsChanged,
         activateOnly](const ManagedObjectType& sensorConfigurations) {
            [[maybe_unused]] bool firstScan = sensorsChanged == nullptr;

            SensorConfigMap configMap =
                buildSensorConfigMap(sensorConfigurations);
            boost::container::flat_map<uint64_t, uint64_t> disabledSensor;
            disabledSensor = checkSysfsAttributesExist(sensorConfigurations);

            // iterate through all found uninstantiateDevices temp sensors,
            // and try to match them with configuration
            for (auto& pair : disabledSensor)
            {
                uint64_t bus = pair.first;
                uint64_t addr = pair.second;
                auto findSensorCfg = configMap.find({bus, addr});
                if (findSensorCfg == configMap.end())
                {
                    continue;
                }

                const std::string& interfacePath =
                    findSensorCfg->second.sensorPath;

                const SensorData& sensorData = findSensorCfg->second.sensorData;
                std::string sensorType = findSensorCfg->second.interface;
                auto pos = sensorType.find_last_of('.');
                if (pos != std::string::npos)
                {
                    sensorType = sensorType.substr(pos + 1);
                }
                const SensorBaseConfigMap& baseConfigMap =
                    findSensorCfg->second.config;

                auto findSensorName = baseConfigMap.find("Name");

                int index = 1;
                if (findSensorName == baseConfigMap.end())
                {
                    std::cerr << "could not determine configuration name for "
                              << "\n";
                    continue;
                }
                std::string sensorName =
                    std::get<std::string>(findSensorName->second);

                std::vector<thresholds::Threshold> sensorThresholds;

                if (!parseThresholdsFromConfig(sensorData, sensorThresholds,
                                               nullptr, &index))
                {
                    std::cerr << "error populating thresholds for "
                              << sensorName << " index " << index << "\n";
                }

                if (sensors.contains(sensorName) && sensors[sensorName])
                {
                    std::cerr
                        << "Sensor already exists, skipping: " << sensorName
                        << "\n";
                    continue;
                }

                auto& sensor = sensors[sensorName];
                std::optional<std::string> hwmonFile = std::nullopt;

                try
                {
                    sensor = std::make_shared<DamagedSensor>(
                        *hwmonFile, sensorType, objectServer, dbusConnection,
                        io, sensorName, std::move(sensorThresholds),
                        interfacePath);
                    sensor->setupRead();
                }
                catch (const std::exception& e)
                {
                    std::cerr << "Failed to create sensor '" << sensorName
                              << "': " << e.what() << std::endl;
                    continue;
                }
            }
        });
    std::vector<std::string> types(sensorTypes.size());
    for (const auto& [type, dt] : sensorTypes)
    {
        types.push_back(type);
    }
    getter->getConfiguration(types);
}

void interfaceRemoved(
    sdbusplus::message_t& message,
    boost::container::flat_map<std::string, std::shared_ptr<DamagedSensor>>&
        sensors)
{
    if (message.is_method_error())
    {
        std::cerr << "interfacesRemoved callback method error\n";
        return;
    }

    sdbusplus::message::object_path path;
    std::vector<std::string> interfaces;

    message.read(path, interfaces);

    // If the xyz.openbmc_project.Confguration.X interface was removed
    // for one or more sensors, delete those sensor objects.
    auto sensorIt = sensors.begin();
    while (sensorIt != sensors.end())
    {
        if (sensorIt->second && (sensorIt->second->configurationPath == path) &&
            (std::find(interfaces.begin(), interfaces.end(),
                       sensorIt->second->configInterface) != interfaces.end()))
        {
            sensorIt = sensors.erase(sensorIt);
        }
        else
        {
            sensorIt++;
        }
    }
}

int main()
{
    try
    {
        boost::asio::io_context io;
        auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
        sdbusplus::asio::object_server objectServer(systemBus, true);
        objectServer.add_manager("/xyz/openbmc_project/sensors");
        systemBus->request_name("xyz.openbmc_project.DamagedSensor");

        boost::container::flat_map<std::string, std::shared_ptr<DamagedSensor>>
            sensors;
        auto sensorsChanged =
            std::make_shared<boost::container::flat_set<std::string>>();

        boost::asio::post(io, [&]() {
            createSensors(io, objectServer, sensors, systemBus, nullptr, false);
        });

        boost::asio::steady_timer filterTimer(io);
        std::function<void(sdbusplus::message_t&)> eventHandler =
            [&](sdbusplus::message_t& message) {
                if (message.is_method_error())
                {
                    std::cerr << "callback method error\n";
                    return;
                }
                sensorsChanged->insert(message.get_path());
                // this implicitly cancels the timer
                filterTimer.expires_after(std::chrono::seconds(1));

                filterTimer.async_wait(
                    [&](const boost::system::error_code& ec) {
                        if (ec == boost::asio::error::operation_aborted)
                        {
                            /* we were canceled*/
                            return;
                        }
                        if (ec)
                        {
                            std::cerr << "timer error\n";
                            return;
                        }
                        createSensors(io, objectServer, sensors, systemBus,
                                      sensorsChanged, false);
                    });
            };

        std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
            setupPropertiesChangedMatches(*systemBus, sensorTypes,
                                          eventHandler);
        setupManufacturingModeMatch(*systemBus);

        // Watch for entity-manager to remove configuration interfaces
        // so the corresponding sensors can be removed.
        auto ifaceRemovedMatch = std::make_unique<sdbusplus::bus::match_t>(
            static_cast<sdbusplus::bus_t&>(*systemBus),
            "type='signal',member='InterfacesRemoved',arg0path='" +
                std::string(inventoryPath) + "/'",
            [&sensors](sdbusplus::message_t& msg) {
                interfaceRemoved(msg, sensors);
            });

        matches.emplace_back(std::move(ifaceRemovedMatch));

        io.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Unhandled exception: " << e.what() << std::endl;
        return -1;
    }
}
