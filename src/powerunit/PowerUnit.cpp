#include <unistd.h>

#include <PowerUnit.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

PowerUnit::PowerUnit(sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_context& io __attribute__((unused)),
                     const std::string& sensorName,
                     const std::string& sensorConfiguration,
                     std::optional<uint8_t> sensorSDRType) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objServer(objectServer)
{
    if (sensorSDRType.has_value() && sensorSDRType.value() == EVENT_SDR_TYPE)
    {
        sensorInterface = objectServer.add_interface(
            baseObj + name, "xyz.openbmc_project.Sensor.EventOnly");
    }
    else
    {
        sensorInterface = objectServer.add_interface(
            baseObj + name, "xyz.openbmc_project.Sensor.State");
    }

    association =
        objectServer.add_interface(baseObj + name, association::interface);
    setInitialProperties();

    // write sensor specific code
    auto powerStatusMatcherCallback = [this, &conn](sdbusplus::message_t& msg) {
        std::cerr << "power state changed\n";
        std::string objectName;
        boost::container::flat_map<std::string, std::variant<std::string>>
            values;

        std::vector<uint8_t> eventData(selEvtDataMaxSize, 0xFF);
        bool assertion = false;

        msg.read(objectName, values);
        auto findState = values.find(powerProperty);
        if (findState != values.end())
        {
            if (std::get<std::string>(findState->second) ==
                "xyz.openbmc_project.State.Host.HostState.Off")
            {
                updateState(sensorInterface, 0x01);
                assertion = true;
            }
            else
            {
                updateState(sensorInterface, 0x00);
                assertion = false;
            }

            std::vector<std::string> logData{name, "Power Down", baseObj + name,
                                             "PowerUnit"};
            eventData[0] = static_cast<uint8_t>(0x00);

            addSelEntry(conn, logData, eventData, assertion);
        }
    };

    powerMonitor =
        setupDbusMatch(powerPath, powerInterface, powerStatusMatcherCallback);
}

PowerUnit::~PowerUnit()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}
