#include <unistd.h>

#include <DigitalDiscrete.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

DigitalDiscrete::DigitalDiscrete(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io __attribute__((unused)),
    const std::string& sensorName, const std::string& sensorConfiguration,
    unsigned int& eventType, unsigned int& subType) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objServer(objectServer), eveType(eventType), subType(subType)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/chassisstate/" + name,
        "xyz.openbmc_project.Sensor.State");

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/chassisstate/" + name,
        association::interface);
    setInitialProperties();
}

DigitalDiscrete::~DigitalDiscrete()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void DigitalDiscrete::setupRead(void)
{

    deviceFunction = {
        {static_cast<unsigned int>(DigitalType::State),
         [this]() { monitorState(); }},
        {static_cast<unsigned int>(DigitalType::Failure),
         [this]() { monitorFailure(); }},
        {static_cast<unsigned int>(DigitalType::Limit),
         [this]() { monitorLimit(); }},
        {static_cast<unsigned int>(DigitalType::Performance),
         [this]() { monitorPerformance(); }},
        {static_cast<unsigned int>(DigitalType::Presence),
         [this]() { monitorPresence(); }},
        {static_cast<unsigned int>(DigitalType::Enabled),
         [this]() { monitorEnabled(); }},
    };

    auto it = deviceFunction.find(eveType);
    if (it != deviceFunction.end())
    {
        it->second();
    }
}

void DigitalDiscrete::monitorState(void)
{

    // write sensor specific code
    auto powerStatusMatcherCallback = [this](sdbusplus::message_t& msg) {
        std::cerr << "power state changed\n";
        std::string objectName;
        boost::container::flat_map<std::string, std::variant<std::string>>
            values;

        std::vector<uint8_t> eventData(selEvtDataMaxSize, 0xFF);
        bool assertion = true;
        std::vector<std::string> logData;
        logData.push_back(name);

        msg.read(objectName, values);
        auto findState = values.find(powerProperty);
        if (findState != values.end())
        {
            if (std::get<std::string>(findState->second) ==
                "xyz.openbmc_project.State.Host.HostState.Off")
            {
                updateState(sensorInterface, 0x01);
                eventData[0] = static_cast<uint8_t>(0x00);
                logData.push_back("State Deasserted");
            }
            else
            {
                updateState(sensorInterface, 0x02);
                eventData[0] = static_cast<uint8_t>(0x01);
                logData.push_back("State Asserted");
            }

            logData.push_back(baseObj + name);
            logData.push_back("DigitalState");

            addSelEntry(dbusConnection, logData, eventData, assertion);
        }
    };

    powerMonitor =
        setupDbusMatch(powerPath, powerInterface, powerStatusMatcherCallback);
}

void DigitalDiscrete::monitorFailure(void)
{
    // Implement code for digital failure sensor
}

void DigitalDiscrete::monitorLimit(void)
{
    // Implement code for digital limit sensor
}

void DigitalDiscrete::monitorPerformance(void)
{
    // Implement code for digital performance sensor
}

void DigitalDiscrete::monitorPresence(void)
{
    // Implement code for digital presence sensor
}

void DigitalDiscrete::monitorEnabled(void)
{
    // Implement code for digital presence sensor
}
