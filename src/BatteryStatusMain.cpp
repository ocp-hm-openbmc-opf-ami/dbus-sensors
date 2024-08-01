#include <BatteryStatus.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/bus/match.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <variant>
#include <vector>

static constexpr const char* sensorType = "Battery";

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<BatteryStatus>>&
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
        const std::pair<std::string, SensorBaseConfigMap>* baseConfiguration =
            nullptr;

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
                std::cerr << "error finding base configuration"
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

            auto findDeviceName = baseConfiguration->second.find("DeviceName");
            if (findDeviceName == baseConfiguration->second.end())
            {
                std::cerr << "could not determine DeviceName from configuration"
                          << "\n";
                continue;
            }
            std::string deviceName =
                std::get<std::string>(findDeviceName->second);

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
            auto& sensorConstruct = sensors[sensorName];
            sensorConstruct = nullptr;

            sensorConstruct = std::make_shared<BatteryStatus>(
                objectServer, dbusConnection, io, sensorName, deviceName,
                *interfacePath);

            sensorConstruct->setupRead();
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
    systemBus->request_name("xyz.openbmc_project.BatteryStatus");

    boost::container::flat_map<std::string, std::shared_ptr<BatteryStatus>>
        sensors;
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, sensors, systemBus, nullptr);
    });

    boost::asio::deadline_timer filterTimer(io);
    std::function<void(sdbusplus::message::message&)> eventHandler =
        [&](sdbusplus::message::message& message) {
        if (message.is_method_error())
        {
            std::cerr << "callback method error\n";
            return;
        }
        sensorsChanged->insert(message.get_path());
        // this implicitly cancels the timer
        filterTimer.expires_from_now(boost::posix_time::seconds(1));
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
            createSensors(io, objectServer, sensors, systemBus, sensorsChanged);
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
