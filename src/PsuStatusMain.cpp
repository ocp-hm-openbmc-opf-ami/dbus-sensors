#include <PsuStatus.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/bus/match.hpp>

#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <variant>
#include <vector>

static constexpr auto sensorTypes{std::to_array<const char*>({"Powersupply"})};

void createSensors(
    boost::asio::io_context& io, sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<PsuStatus>>&
        sensors,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    auto getter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &objectServer, &sensors, &dbusConnection,
         sensorsChanged](const ManagedObjectType& sensorConfigs) {
        bool firstScan = sensorsChanged == nullptr;
        std::vector<fs::path> pmbusPaths;
        if (!findFiles(fs::path("/sys/class/hwmon"), "name", pmbusPaths))
        {
            std::cerr << "No PSU sensors in system\n";
            return;
        }
        boost::container::flat_set<std::string> directories;
        for (const auto& pmbusPath : pmbusPaths)
        {
            std::ifstream nameFile(pmbusPath);
            if (!nameFile.good())
            {
                std::cerr << " Failure finding power supply path \n";
                continue;
            }

            std::string pmbusName;
            std::getline(nameFile, pmbusName);
            nameFile.close();

            if (pmbusName != "pmbus")
            {
                continue;
            }

            auto directory = pmbusPath.parent_path();

            auto ret = directories.insert(directory.string());
            if (!ret.second)
            {
                std::cerr << " Duplicate path " << directory.string() << "\n";
                continue;
            }
            fs::path device = directory / "device";
            std::string deviceName = fs::canonical(device).stem();
            auto findHyphen = deviceName.find('-');
            if (findHyphen == std::string::npos)
            {
                std::cerr << " Found bad device " << deviceName << "\n";
                continue;
            }

            std::string busStr = deviceName.substr(0, findHyphen);
            std::string addrStr = deviceName.substr(findHyphen + 1);

            size_t bus = 0;
            size_t addr = 0;

            try
            {
                bus = std::stoi(busStr);
                addr = std::stoi(addrStr, nullptr, 16);
            }

            catch (const std::invalid_argument&)
            {
                std::cerr << " Error parsing bus " << busStr << " addr "
                          << addrStr << "\n";
                continue;
            }
            const SensorBaseConfigMap* baseConfig = nullptr;
            const SensorData* sensorData = nullptr;
            const std::string* interfacePath = nullptr;
            for (const auto& [path, cfgData] : sensorConfigs)
            {
                sensorData = &cfgData;
                for (const char* type : sensorTypes)
                {
                    auto sensorBase =
                        sensorData->find(configInterfaceName(type));
                    if (sensorBase != sensorData->end())
                    {
                        baseConfig = &sensorBase->second;
                        break;
                    }
                }
                interfacePath = &path.str;

                if (baseConfig == nullptr)
                {
                    std::cerr << " error finding base configuration for "
                              << deviceName << "\n";
                    continue;
                }
                auto configBus = baseConfig->find("Bus");
                auto configAddress = baseConfig->find("Address");

                if (configBus == baseConfig->end() ||
                    configAddress == baseConfig->end())
                {
                    std::cerr
                        << "error finding necessary entry in configuration\n";
                    continue;
                }
                const uint64_t* confBus =
                    std::get_if<uint64_t>(&(configBus->second));
                const uint64_t* confAddr =
                    std::get_if<uint64_t>(&(configAddress->second));
                if (confBus == nullptr || confAddr == nullptr)
                {
                    std::cerr << " Cannot get bus or address, invalid "
                                 "configuration \n";
                    continue;
                }
                if ((*confBus != bus) || (*confAddr != addr))
                {
                    std::cerr << "Skipping as configuration not matching\n";
                    continue;
                }
                break;
            }

            auto findSensorName = baseConfig->find("Name");
            if (findSensorName == baseConfig->end())
            {
                std::cerr << " could not determine config name for "
                          << deviceName << "\n";
                continue;
            }
            std::string sensorName =
                std::get<std::string>(findSensorName->second);

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
            boost::container::flat_map<std::string, std::vector<std::string>>
                pathList;
            sensorConstruct = std::make_shared<PsuStatus>(
                objectServer, dbusConnection, io, sensorName, directory,
                pathList, *interfacePath);
            sensorConstruct->setupRead();
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
    systemBus->request_name("xyz.openbmc_project.PsuStatus");
    boost::container::flat_map<std::string, std::shared_ptr<PsuStatus>> sensors;
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
        filterTimer.expires_from_now(boost::posix_time::seconds(5));

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
