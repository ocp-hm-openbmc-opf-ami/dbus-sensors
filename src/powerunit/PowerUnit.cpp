#include <unistd.h>

#include <PowerUnit.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

PowerUnit::PowerUnit(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io __attribute__((unused)),
    const std::string& sensorName, const std::string& sensorConfiguration,
    std::optional<uint8_t> sensorSDRType, uint16_t sensorNumber, uint8_t lun) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn, sensorNumber,
             lun),
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
    auto powerStatusMatcherCallback = [this, &conn, sensorNumber](
                                          sdbusplus::message_t& msg) {
        std::string objectName;
        boost::container::flat_map<std::string, std::variant<std::string>>
            values;

        std::vector<uint8_t> eventData(selEvtDataMaxSize, 0xFF);
        bool assertion = false;

        msg.read(objectName, values);
        auto findState = values.find(hostProperty);
        auto powerState = values.find(chassisProperty);
        if (powerState != values.end())
        {
            std::string transitions = std::get<std::string>(powerState->second);
            if (transitions ==
                "xyz.openbmc_project.State.Chassis.PowerState.Off")
            {
                updateState(sensorInterface,
                            (static_cast<uint8_t>(PowerUnitEvent::powerDown)));
                assertion = true;
                std::vector<std::string> logData{name, "Power Down",
                                                 baseObj + name,
                                                 "SensorDevicePowerUnitAssert"};
                eventData[0] = 0x00;
                addSelEntry(conn, logData, eventData, assertion, sensorNumber);
            }
        }
        else if (findState != values.end())
        {
            std::string transition = std::get<std::string>(findState->second);
            if (transition ==
                "xyz.openbmc_project.State.Host.Transition.Reboot")
            {
                updateState(sensorInterface,
                            (static_cast<uint8_t>(PowerUnitEvent::powerCycle)));
                assertion = true;

                std::vector<std::string> logData{name, "Power Cycle",
                                                 baseObj + name,
                                                 "SensorDevicePowerUnitAssert"};

                eventData[0] = 0x01;
                addSelEntry(conn, logData, eventData, assertion, sensorNumber);
            }
        }
    };
    powerCycleMonitor =
        setupDbusMatch(hostPath, hostInterface, powerStatusMatcherCallback);
    powerOffMonitor = setupDbusMatch(chassisPath, chassisInterface,
                                     powerStatusMatcherCallback);
}

PowerUnit::~PowerUnit()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}
