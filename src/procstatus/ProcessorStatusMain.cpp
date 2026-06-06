#include <ProcessorStatus.hpp>
#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/bus/match.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <variant>
#include <vector>

static constexpr const char* sensorType = "Cpustatus";

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<ProcessorStatus>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &objectServer, &sensors, &dbusConnection,
         sensorsChanged](const ManagedObjectType& sensorConfigurations) {
            bool firstScan = sensorsChanged == nullptr;
            const std::string* interfacePath = nullptr;
            const std::pair<std::string, SensorBaseConfigMap>*
                baseConfiguration = nullptr;

            for (const auto& [path, cfgData] : sensorConfigurations)
            {
                // clear it out each loop
                baseConfiguration = nullptr;
                auto sensorBase = cfgData.find(configInterfaceName(sensorType));
                if (sensorBase == cfgData.end())
                {
                    continue;
                }
                baseConfiguration = &(*sensorBase);
                interfacePath = &path.str;

                if (baseConfiguration == nullptr)
                {
                    std::cerr
                        << "error finding base configuration for sensor types"
                        << "\n";
                    continue;
                }
                auto findSensorName = baseConfiguration->second.find("Name");
                if (findSensorName == baseConfiguration->second.end())
                {
                    std::cerr << "could not determine configuration name"
                              << "\n";
                    continue;
                }
                std::string sensorName =
                    std::get<std::string>(findSensorName->second);

                uint16_t sensorNumber = defaultSensorNumber;
                uint8_t lun = defaultLun;
                auto findSensorNum =
                    baseConfiguration->second.find("SensorNumber");
                if (findSensorNum != baseConfiguration->second.end())
                {
                    try
                    {
                        sensorNumber = static_cast<uint16_t>(
                            std::visit(VariantToUnsignedIntVisitor(),
                                       findSensorNum->second));
                    }
                    catch (const std::exception&)
                    {
                        std::cerr << "Invalid SensorNumber for " << sensorName
                                  << "\n";
                        sensorNumber = defaultSensorNumber;
                    }
                }

                auto findLun = baseConfiguration->second.find("LUN");
                if (findLun != baseConfiguration->second.end())
                {
                    try
                    {
                        lun = static_cast<uint8_t>(std::visit(
                            VariantToUnsignedIntVisitor(), findLun->second));
                    }
                    catch (const std::exception&)
                    {
                        std::cerr << "Invalid LUN for " << sensorName << "\n";
                    }
                }

                bool dbusFound = false;
                bool gpioFound = false;
                std::vector<std::string> gpioNames;
                std::vector<std::string> dbusPaths, dbusIfaces, dbusProperties;

                auto findGpioName = baseConfiguration->second.find("GpioName");
                if (findGpioName != baseConfiguration->second.end())
                {
                    std::vector<std::string> gpioList =
                        std::visit(VariantNamesVisitor(), findGpioName->second);
                    gpioNames = gpioList;
                    for (const auto& gpio : gpioList)
                    {
                        if (!gpio.empty())
                        {
                            gpioFound = true;
                            break;
                        }
                    }
                }

                auto dbusPathFound = baseConfiguration->second.find("DBusPath");
                auto dbusIfaceFound =
                    baseConfiguration->second.find("DBusIface");
                auto dbusPropNameFound =
                    baseConfiguration->second.find("DBusProperty");

                if (dbusPathFound != baseConfiguration->second.end() &&
                    dbusIfaceFound != baseConfiguration->second.end() &&
                    dbusPropNameFound != baseConfiguration->second.end())
                {
                    dbusPaths = std::visit(VariantNamesVisitor(),
                                           dbusPathFound->second);
                    dbusIfaces = std::visit(VariantNamesVisitor(),
                                            dbusIfaceFound->second);
                    dbusProperties = std::visit(VariantNamesVisitor(),
                                                dbusPropNameFound->second);

                    for (const auto& path : dbusPaths)
                    {
                        if (!path.empty())
                        {
                            dbusFound = true;
                            break;
                        }
                    }
                }

                if (!gpioFound && !dbusFound)
                {
                    continue;
                }
                // on rescans, only update sensors we were signaled by
                auto findSensor = sensors.find(sensorName);
                if (!firstScan && findSensor != sensors.end())
                {
                    bool found = false;
                    for (auto it = sensorsChanged->begin();
                         it != sensorsChanged->end(); it++)
                    {
                        if (findSensor->second &&
                            boost::ends_with(*it, findSensor->second->name))
                        {
                            sensorsChanged->erase(it);
                            findSensor->second = nullptr;
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        continue;
                    }
                }
                std::string polarity;
                if (gpioFound)
                {
                    auto findPolarity =
                        baseConfiguration->second.find("Polarity");
                    if (findPolarity == baseConfiguration->second.end())
                    {
                        std::cerr
                            << "could not determine configuration polarity"
                            << "\n";
                        continue;
                    }
                    polarity = std::get<std::string>(findPolarity->second);
                }
                auto& sensorConstruct = sensors[sensorName];
                sensorConstruct = nullptr;

                sensorConstruct = std::make_shared<ProcessorStatus>(
                    objectServer, dbusConnection, io, sensorName, gpioNames,
                    *interfacePath, dbusPaths, dbusIfaces, dbusProperties,
                    dbusFound, sensorNumber, lun);
            }
        });

    getter->getConfiguration(std::vector<std::string>{sensorType});
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.ProcessorStatus");
    boost::container::flat_map<std::string, std::shared_ptr<ProcessorStatus>>
        sensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    //  io.post([&]() {
    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, sensors, systemBus, nullptr);
    });

    boost::asio::steady_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }
            sensorsChanged->insert(message.get_path());
            // this implicitly cancels the timer
            filterTimer.expires_after(std::chrono::seconds(1));

            filterTimer.async_wait([&](const boost::system::error_code& ec) {
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
                              sensorsChanged);
            });
        };

    auto match = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(inventoryPath) + "',arg0namespace='" +
            configInterfaceName(sensorType) + "'",
        eventHandler);
    matches.emplace_back(std::move(match));
    io.run();
}
