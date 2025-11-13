/*
// Copyright (c) 2022 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "DBusADCSensor.hpp"

#include <Utils.hpp>
#include <VariantVisitors.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

static constexpr float defaultPollRate = 1.0;
static constexpr float defaultGpioBridgeSetupTime = 0.02;
static constexpr float defaultScaleFactor = 1.0;

static constexpr auto sensorTypes{std::to_array<const char*>({"DBusADC"})};

static constexpr auto controllerTypes{
    std::to_array<const char*>({"DBusADCController"})};

static boost::container::flat_map<size_t, bool> cpuPresence;

void createControllers(
    boost::asio::io_context& io,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<
        std::string, std::shared_ptr<DBusADCSensorController>>& controllers,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        controllersChanged,
    boost::container::flat_map<std::string, std::shared_ptr<DBusADCSensor>>&
        sensors,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged);

void createSensors(
    boost::asio::io_context& io,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<DBusADCSensor>>&
        sensors,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    boost::container::flat_map<
        std::string, std::shared_ptr<DBusADCSensorController>>& controllers);

static void getPresentCpus(
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection)
{
    GetSubTreeType cpuSubTree;
    auto getItems = dbusConnection->new_method_call(
        mapper::busName, mapper::path, mapper::interface, mapper::subtree);
    getItems.append(
        cpuInventoryPath, static_cast<int32_t>(2),
        std::array<const char*, 1>{"xyz.openbmc_project.Inventory.Item"});

    try
    {
        auto getItemsResp = dbusConnection->call(getItems);
        getItemsResp.read(cpuSubTree);
    }
    catch (sdbusplus::exception_t&)
    {
        std::cerr << "error getting inventory item subtree\n";
        return;
    }

    for (const auto& [path, objDict] : cpuSubTree)
    {
        auto obj = sdbusplus::message::object_path(path).filename();
        if (!obj.starts_with("cpu") || objDict.empty())
        {
            continue;
        }
        const std::string& owner = objDict.begin()->first;

        auto getPresence = dbusConnection->new_method_call(
            owner.c_str(), path.c_str(), "org.freedesktop.DBus.Properties",
            "Get");

        getPresence.append("xyz.openbmc_project.Inventory.Item", "Present");
        std::variant<bool> respValue;
        try
        {
            auto resp = dbusConnection->call(getPresence);
            resp.read(respValue);
        }
        catch (sdbusplus::exception_t&)
        {
            std::cerr << "Error in getting CPU presence\n";
            continue;
        }

        auto present = std::get_if<bool>(&respValue);
        if (present != nullptr && *present)
        {
            int cpuIndex;
            try
            {
                cpuIndex = std::stoi(obj.substr(obj.find_last_of("cpu") + 1));
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error converting CPU index, " << e.what() << '\n';
                continue;
            }
            cpuPresence[cpuIndex + 1] = *present;
        }
    }
}

int main()
{
    boost::asio::io_context io;
    boost::asio::signal_set signals =
        boost::asio::signal_set(io, SIGINT, SIGTERM);
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    systemBus->request_name("xyz.openbmc_project.DBusADCSensor");
    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    boost::container::flat_map<std::string, std::shared_ptr<DBusADCSensor>>
        sensors;
    auto sensorsChanged =
        std::make_shared<boost::container::flat_set<std::string>>();
    boost::container::flat_map<std::string,
                               std::shared_ptr<DBusADCSensorController>>
        controllers;
    auto controllersChanged =
        std::make_shared<boost::container::flat_set<std::string>>();
    std::vector<std::unique_ptr<sdbusplus::bus::match::match>> matches;

    signals.async_wait(
        [&io, &controllers](const boost::system::error_code ec, const int&) {
            if (ec)
            {
                throw std::runtime_error("Signal should not be cancelled.");
            }

            for (const auto& [name, controller] : controllers)
            {
                controller->disable();
            }

            io.stop();
        });

    boost::asio::post(io, [&io, &systemBus, &objectServer, &controllers,
                           &controllersChanged, &sensors, &sensorsChanged]() {
        createControllers(io, systemBus, objectServer, controllers,
                          controllersChanged, sensors, sensorsChanged);
        createSensors(io, systemBus, objectServer, sensors, sensorsChanged,
                      controllers);
    });

    boost::asio::deadline_timer sensorFilterTimer(io);
    std::function<void(sdbusplus::message::message&)> sensorEventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }

            auto it = std::find_if(
                sensors.cbegin(), sensors.cend(), [&](const auto& elem) {
                    return boost::ends_with(message.get_path(),
                                            elem.second->name);
                });

            if (it != sensors.cend())
            {
                sensorsChanged->insert(message.get_path());
            }

            sensorFilterTimer.expires_from_now(boost::posix_time::seconds(1));

            sensorFilterTimer.async_wait(
                [&](const boost::system::error_code& ec) {
                    if (ec)
                    {
                        if (ec != boost::asio::error::operation_aborted)
                        {
                            std::cerr << "timer error\n";
                        }
                        return;
                    }
                    createSensors(io, systemBus, objectServer, sensors,
                                  sensorsChanged, controllers);
                });
        };

    boost::asio::deadline_timer controllerFilterTimer(io);
    std::function<void(sdbusplus::message::message&)> controllerEventHandler =
        [&](sdbusplus::message::message& message) {
            if (message.is_method_error())
            {
                std::cerr << "callback method error\n";
                return;
            }

            auto it =
                std::find_if(controllers.cbegin(), controllers.cend(),
                             [&](const auto& elem) {
                                 return boost::ends_with(message.get_path(),
                                                         elem.second->name);
                             });

            if (it != controllers.cend())
            {
                controllersChanged->insert(message.get_path());
            }

            controllerFilterTimer.expires_from_now(
                boost::posix_time::seconds(1));

            controllerFilterTimer.async_wait(
                [&](const boost::system::error_code& ec) {
                    if (ec)
                    {
                        if (ec != boost::asio::error::operation_aborted)
                        {
                            std::cerr << "timer error\n";
                        }
                        return;
                    }
                    createControllers(io, systemBus, objectServer, controllers,
                                      controllersChanged, sensors,
                                      sensorsChanged);
                });
        };

    boost::asio::deadline_timer cpuFilterTimer(io);
    std::function<void(sdbusplus::message::message&)> cpuPresenceHandler =
        [&](sdbusplus::message::message& message) {
            std::string path = message.get_path();
            boost::to_lower(path);

            if (path.rfind("cpu") == std::string::npos)
            {
                return;
            }
            size_t index = 0;
            try
            {
                index = std::stoi(path.substr(path.size() - 1));
            }
            catch (const std::invalid_argument&)
            {
                std::cerr << "Found invalid path " << path << "\n";
                return;
            }

            std::string objectName;
            boost::container::flat_map<std::string, std::variant<bool>> values;
            message.read(objectName, values);
            auto findPresence = values.find("Present");
            if (findPresence == values.end())
            {
                return;
            }
            cpuPresence[index] = std::get<bool>(findPresence->second);
            cpuFilterTimer.expires_from_now(boost::posix_time::seconds(1));
            cpuFilterTimer.async_wait([&](const boost::system::error_code& ec) {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return;
                }
                if (ec)
                {
                    std::cerr << "timer error\n";
                    return;
                }

                createSensors(io, systemBus, objectServer, sensors,
                              sensorsChanged, controllers);
            });
        };

    for (const char* type : sensorTypes)
    {
        auto match = std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*systemBus),
            "type='signal',member='PropertiesChanged',path_namespace='" +
                std::string(inventoryPath) + "',arg0namespace='" +
                configInterfaceName(type) + "'",
            sensorEventHandler);
        matches.emplace_back(std::move(match));
    }
    for (const char* type : controllerTypes)
    {
        auto match = std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*systemBus),
            "type='signal',member='PropertiesChanged',path_namespace='" +
                std::string(inventoryPath) + "',arg0namespace='" +
                configInterfaceName(type) + "'",
            controllerEventHandler);
        matches.emplace_back(std::move(match));
    }

    matches.emplace_back(std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*systemBus),
        "type='signal',member='PropertiesChanged',path_namespace='" +
            std::string(cpuInventoryPath) +
            "',arg0namespace='xyz.openbmc_project.Inventory.Item'",
        cpuPresenceHandler));

    getPresentCpus(systemBus);

    setupManufacturingModeMatch(*systemBus);
    io.run();
    return 0;
}

template <typename Config, typename T>
bool findConfiguration(Config** config, const SensorData& data, const T& types)
{
    for (const char* type : types)
    {
        auto cfg = data.find(configInterfaceName(type));
        if (cfg != data.end())
        {
            *config = &(*cfg);
            return true;
        }
    }
    return false;
}

template <typename EntitiesChanged, typename Entities>
bool entityChanged(const std::string& name, Entities& entities,
                   EntitiesChanged& entitiesChanged)
{
    if (auto entityFound = entities.find(name); entityFound != entities.end())
    {
        for (const auto& entityChanged : *entitiesChanged)
        {
            if (entityFound->second &&
                boost::ends_with(entityChanged, entityFound->second->name))
            {
                entitiesChanged->erase(entityChanged);
                entityFound->second = nullptr;

                return true;
            }
        }
    }

    return false;
}

void createControllers(
    boost::asio::io_context& io,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<
        std::string, std::shared_ptr<DBusADCSensorController>>& controllers,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        controllersChanged,
    boost::container::flat_map<std::string, std::shared_ptr<DBusADCSensor>>&
        sensors,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged)
{
    auto controllerGetter = std::make_shared<GetSensorConfiguration>(
        dbusConnection,
        [&io, &dbusConnection, &objectServer, &controllers, controllersChanged,
         &sensors,
         sensorsChanged](const ManagedObjectType& controllerConfigurations) {
            bool anyControllerChanged = false;

            for (const auto& [objPath, data] : controllerConfigurations)
            {
                const std::pair<
                    std::string,
                    boost::container::flat_map<std::string, BasicVariantType>>*
                    configuration = nullptr;

                if (!findConfiguration(&configuration, data, controllerTypes))
                {
                    continue;
                }
                auto nameFound = configuration->second.find("Name");
                if (nameFound == configuration->second.end())
                {
                    std::cerr << "Missing mandatory Name property for: "
                              << configuration->first << " object\n";
                    continue;
                }
                auto name =
                    std::visit(VariantToStringVisitor(), nameFound->second);

                if (!controllersChanged->empty())
                {
                    bool controllerChanged =
                        entityChanged(name, controllers, controllersChanged);

                    if (!controllerChanged)
                    {
                        continue;
                    }
                    anyControllerChanged |= controllerChanged;
                }

                auto dbusServiceFound =
                    configuration->second.find("DBusService");
                if (dbusServiceFound == configuration->second.end())
                {
                    std::cerr << "Missing mandatory DBusService property for: "
                              << configuration->first << " object\n";
                    continue;
                }
                auto dbusService = std::visit(VariantToStringVisitor(),
                                              dbusServiceFound->second);

                auto dbusPathFound = configuration->second.find("DBusPath");
                if (dbusPathFound == configuration->second.end())
                {
                    std::cerr << "Missing mandatory DBusPath property for: "
                              << configuration->first << " object\n";
                    continue;
                }
                auto dbusPath =
                    std::visit(VariantToStringVisitor(), dbusPathFound->second);

                auto dbusIfaceFound = configuration->second.find("DBusIface");
                if (dbusIfaceFound == configuration->second.end())
                {
                    std::cerr << "Missing mandatory DBusIface property for: "
                              << configuration->first << " object\n";
                    continue;
                }
                auto dbusIface = std::visit(VariantToStringVisitor(),
                                            dbusIfaceFound->second);

                auto vrefFound = configuration->second.find("VREF");
                if (vrefFound == configuration->second.end())
                {
                    std::cerr << "Missing mandatory VREF property for: "
                              << configuration->first << " object\n";
                    continue;
                }
                auto vref =
                    std::visit(VariantToFloatVisitor(), vrefFound->second);

                auto resolutionFound =
                    configuration->second.find("ResolutionBits");
                if (resolutionFound == configuration->second.end())
                {
                    std::cerr
                        << "Missing mandatory ResolutionBits property for: "
                        << configuration->first << " object\n";
                    continue;
                }
                auto resolution = std::visit(VariantToUnsignedIntVisitor(),
                                             resolutionFound->second);

                controllers[name] = nullptr;

                auto controller = std::make_shared<DBusADCSensorController>(
                    dbusConnection, name, std::move(dbusService),
                    std::move(dbusPath), std::move(dbusIface), vref,
                    resolution);

                if (controller->enable())
                {
                    controllers[name] = std::move(controller);
                }
            }

            if (anyControllerChanged)
            {
                sensorsChanged->clear();
                sensors.clear();

                createSensors(io, dbusConnection, objectServer, sensors,
                              sensorsChanged, controllers);
            }
        });

    controllerGetter->getConfiguration(std::vector<std::string>{
        controllerTypes.begin(), controllerTypes.end()});
}

void createSensors(
    boost::asio::io_context& io,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    sdbusplus::asio::object_server& objectServer,
    boost::container::flat_map<std::string, std::shared_ptr<DBusADCSensor>>&
        sensors,
    const std::shared_ptr<boost::container::flat_set<std::string>>&
        sensorsChanged,
    boost::container::flat_map<
        std::string, std::shared_ptr<DBusADCSensorController>>& controllers)
{
    auto sensorGetter = std::make_shared<
        GetSensorConfiguration>(dbusConnection, [&io, &dbusConnection,
                                                 &objectServer, &sensors,
                                                 sensorsChanged, &controllers](
                                                    const ManagedObjectType&
                                                        sensorConfigurations) {
        for (const auto& [objPath, data] : sensorConfigurations)
        {
            const std::pair<std::string, boost::container::flat_map<
                                             std::string, BasicVariantType>>*
                configuration = nullptr;

            if (!findConfiguration(&configuration, data, sensorTypes))
            {
                continue;
            }

            auto nameFound = configuration->second.find("Name");
            if (nameFound == configuration->second.end())
            {
                std::cerr << "Missing mandatory Name property for: "
                          << configuration->first << " object\n";
                continue;
            }
            auto name = std::visit(VariantToStringVisitor(), nameFound->second);

            auto controllerNameFound =
                configuration->second.find("ControllerName");
            if (controllerNameFound == configuration->second.end())
            {
                std::cerr << "Missing mandatory ControllerName property for: "
                          << configuration->first << " object\n";
                continue;
            }
            auto controllerName = std::visit(VariantToStringVisitor(),
                                             controllerNameFound->second);

            auto controller =
                std::find_if(controllers.cbegin(), controllers.cend(),
                             [controllerName](const auto& elem) {
                                 return controllerName == elem.second->name;
                             });
            if (controller == controllers.cend())
            {
                std::cerr << "Couldn't find controller mapped to sensor: "
                          << name << '\n';
                continue;
            }
            if (!controller->second->isEnabled())
            {
                std::cerr
                    << "Controller failed to enable itself. Skipping sensor: "
                    << name << '\n';
                continue;
            }

            if (!sensorsChanged->empty() &&
                !entityChanged(name, sensors, sensorsChanged))
            {
                continue;
            }

            auto cpuRequirementFound =
                configuration->second.find("CPURequired");
            if (cpuRequirementFound != configuration->second.end())
            {
                size_t cpuIdx = std::visit(VariantToIntVisitor(),
                                           cpuRequirementFound->second);
                auto cpuPresenceFound = cpuPresence.find(cpuIdx);
                if (cpuPresenceFound == cpuPresence.end() ||
                    cpuPresenceFound->second == false)
                {
                    continue;
                }
            }

            auto dbusPropNameFound = configuration->second.find("DBusProperty");
            if (dbusPropNameFound == configuration->second.end())
            {
                std::cerr << "Missing mandatory DBusProperty property for: "
                          << configuration->first << " object\n";
                continue;
            }
            auto dbusPropName =
                std::visit(VariantToStringVisitor(), dbusPropNameFound->second);

            auto scaleFactorFound = configuration->second.find("ScaleFactor");
            float scaleFactor = defaultScaleFactor;
            if (scaleFactorFound != configuration->second.end())
            {
                scaleFactor = std::visit(VariantToFloatVisitor(),
                                         scaleFactorFound->second);
                if (scaleFactor == 0.0f)
                {
                    scaleFactor = defaultScaleFactor;
                }
            }

            auto powerStateFound = configuration->second.find("PowerState");
            PowerState readAtState = PowerState::always;
            if (powerStateFound != configuration->second.end())
            {
                std::string powerState = std::visit(VariantToStringVisitor(),
                                                    powerStateFound->second);
                setReadState(powerState, readAtState);
            }

            std::vector<thresholds::Threshold> sensorThresholds;
            if (!parseThresholdsFromConfig(data, sensorThresholds))
            {
                std::cerr << "Error populating thresholds for: "
                          << configuration->first << " object\n";
            }

            auto pollRateFound = configuration->second.find("PollRate");
            float pollRate = defaultPollRate;
            if (pollRateFound != configuration->second.end())
            {
                pollRate =
                    std::visit(VariantToFloatVisitor(), pollRateFound->second);
                if (pollRate <= 0.0f)
                {
                    pollRate = defaultPollRate;
                }
            }

            std::optional<BridgeGpio> bridgeGpio;
            for (const SensorBaseConfiguration& suppConfig : data)
            {
                if (suppConfig.first.find("BridgeGpio") != std::string::npos)
                {
                    auto findName = suppConfig.second.find("Name");
                    if (findName != suppConfig.second.end())
                    {
                        std::string gpioName = std::visit(
                            VariantToStringVisitor(), findName->second);

                        int polarity = gpiod::line::ACTIVE_HIGH;
                        auto findPolarity = suppConfig.second.find("Polarity");
                        if (findPolarity != suppConfig.second.end())
                        {
                            if (std::string("Low") ==
                                std::visit(VariantToStringVisitor(),
                                           findPolarity->second))
                            {
                                polarity = gpiod::line::ACTIVE_LOW;
                            }
                        }

                        float setupTime = defaultGpioBridgeSetupTime;
                        auto findSetupTime =
                            suppConfig.second.find("SetupTime");
                        if (findSetupTime != suppConfig.second.end())
                        {
                            setupTime = std::visit(VariantToFloatVisitor(),
                                                   findSetupTime->second);
                        }

                        bridgeGpio = BridgeGpio(gpioName, polarity, setupTime);
                    }

                    break;
                }
            }

            auto& sensor = sensors[name];
            sensor = nullptr;

            sensor = std::make_shared<DBusADCSensor>(
                objectServer, dbusConnection, io, name,
                std::move(sensorThresholds), controller->second->dbusService,
                controller->second->dbusObjectPath,
                controller->second->dbusIface, std::move(dbusPropName),
                controller->second->getVref(),
                controller->second->getResolution(), scaleFactor, pollRate,
                readAtState, objPath.str, std::move(bridgeGpio));

            sensor->setupRead();
        }
    });

    sensorGetter->getConfiguration(
        std::vector<std::string>{sensorTypes.begin(), sensorTypes.end()});
}
