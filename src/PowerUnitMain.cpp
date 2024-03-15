#include <PowerUnit.hpp>
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

// static constexpr const char* sensorType = "Powerunit";
static constexpr auto sensorTypes{std::to_array<const char*>({"Powerunit"})};

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<PowerUnit>>&
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
            // find base configuration
            for (const char* type : sensorTypes)
            {
                auto sensorBase = cfgData.find(configInterfaceName(type));
                if (sensorBase != cfgData.end())
                {
                    baseConfiguration = &(*sensorBase);
                    break;
                }
            }
            interfacePath = &path.str;

            if (baseConfiguration == nullptr)
            {
                std::cerr << "error finding base configuration for sensor types"
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

            sensorConstruct = std::make_shared<PowerUnit>(
                objectServer, dbusConnection, io, sensorName, *interfacePath);
        }
    });

    getter->getConfiguration(
        std::vector<std::string>{sensorTypes.begin(), sensorTypes.end()});
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    systemBus->request_name("xyz.openbmc_project.PowerUnit");
    boost::container::flat_map<std::string, std::shared_ptr<PowerUnit>> sensors;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();

    boost::asio::post(io, [&]() {
        createSensors(io, objectServer, sensors, systemBus, nullptr);
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

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*systemBus, sensorTypes, eventHandler);

    io.run();
}
