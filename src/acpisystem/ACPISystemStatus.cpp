#include <unistd.h>

#include <ACPISystemStatus.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

ACPISystemStatus::ACPISystemStatus(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& sensorName, uint16_t sensorNumber, uint8_t lun,
    const std::string& sensorConfiguration) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn, sensorNumber,
             lun),
    objServer(objectServer), sensorNumber(sensorNumber), lun(lun), conn(conn)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/acpisystem/" + name,
        "xyz.openbmc_project.Sensor.State");

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/acpisystem/" + name,
        association::interface);
    setInitialProperties();

    monitorState(conn);
}

ACPISystemStatus::~ACPISystemStatus()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void ACPISystemStatus::propertyInitialize(
    std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    std::vector<uint8_t> eventData(selEvtDataMaxSize, 0xFF);
    std::vector<std::string> logData(logDataMaxSize);

    const std::string objPath =
        "/xyz/openbmc_project/sensors/acpisystem/" + name;
    // At present, the LEGACY_OFF state is disabled.
    // Activation will be considered based on future requirements.
    /*
    reading = reading | (1 << static_cast<uint16_t>(ACPI::LEGACY_OFF));
    logData[0] = name;
    logData[1] = "LEGACY_OFF";
    logData[2] = objPath;
    logData[3] = "SensorSystemACPIPowerState";
    eventData[0] = static_cast<uint8_t>(ACPI::LEGACY_OFF);
    addSelEntry(conn, logData, eventData, true, sensorNumber);
    */

    if (getHostStatus(conn) == static_cast<uint8_t>(HostState::Running))
    {
        eventData[0] = static_cast<uint8_t>(ACPI::S0_G0);
        logData[1] = "S0_G0";
        reading = reading | (1 << static_cast<uint16_t>(ACPI::S0_G0));
    }
    else if (getHostStatus(conn) == static_cast<uint8_t>(HostState::Off))
    {
        eventData[0] = static_cast<uint8_t>(ACPI::S4_S5);
        logData[1] = "S4_S5";
        reading = reading | (1 << static_cast<uint16_t>(ACPI::S4_S5));
    }
    else
    {
        eventData[0] = static_cast<uint8_t>(ACPI::Unknown);
        logData[1] = "Unknown";
        reading = reading | (1 << static_cast<uint16_t>(ACPI::Unknown));
    }
    addSelEntry(conn, logData, eventData, true, sensorNumber);
    updateState(sensorInterface, reading);
}

void ACPISystemStatus::monitorState(
    std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    propertyInitialize(conn);

    auto powerStatusMatcherCallback = [this, &conn](sdbusplus::message_t& msg) {
        std::cerr << "power state changed\n";
        std::string objectName;
        boost::container::flat_map<std::string, std::variant<std::string>>
            values;
        uint16_t oldValue = reading;
        reading = 0;
        const std::string objPath =
            "/xyz/openbmc_project/sensors/acpisystem/" + name;
        std::vector<uint8_t> eventData(selEvtDataMaxSize, 0xFF);
        std::vector<std::string> logData(logDataMaxSize);
        msg.read(objectName, values);
        auto findState = values.find(power::property);
        if (findState != values.end())
        {
            if (std::get<std::string>(findState->second) ==
                "xyz.openbmc_project.State.Host.HostState.Running")
            {
                reading = reading | (1 << static_cast<uint16_t>(ACPI::S0_G0));
                eventData[0] = static_cast<uint8_t>(ACPI::S0_G0);
                logData[1] = "S0_G0";
                std::cerr << "chassis powered on\n";
            }

            else if (std::get<std::string>(findState->second) ==
                     "xyz.openbmc_project.State.Host.HostState.Off")
            {
                eventData[0] = static_cast<uint8_t>(ACPI::S4_S5);
                reading = reading | (1 << static_cast<uint16_t>(ACPI::S4_S5));
                logData[1] = "S4_S5";
                std::cerr << "chassis powered off\n";
            }
            else
            {
                reading = reading | (1 << static_cast<uint16_t>(ACPI::Unknown));
                eventData[0] = static_cast<uint8_t>(ACPI::Unknown);
                logData[1] = "Unknown";
                std::cerr << "chassis unknown state\n";
            }
            if (oldValue != reading)
            {
                logData[0] = name;
                logData[2] = objPath;
                logData[3] = "SensorSystemACPIPowerState";
                addSelEntry(conn, logData, eventData, true, sensorNumber);
                updateState(sensorInterface, reading);
            }
        }
    };

    powerMonitor = setupDbusMatch(power::path, power::interface,
                                  powerStatusMatcherCallback);
}
