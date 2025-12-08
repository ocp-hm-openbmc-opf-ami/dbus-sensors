#include <unistd.h>

#include <LoggingStatus.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

EventStatus::EventStatus(sdbusplus::asio::object_server& objectServer,
                         std::shared_ptr<sdbusplus::asio::connection>& conn,
                         const std::string& sensorName,
                         const std::string& sensorConfiguration,
                         const uint16_t& testLimit) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objServer(objectServer), conn(conn), maxEntries(testLimit),
    objectPath("/xyz/openbmc_project/sensors/logging/" + name)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/logging/" + name,
        "xyz.openbmc_project.Sensor.State");
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/logging/" + name, association::interface);

    setInitialProperties();
    setupRead(conn);
    getEntryCount();
    checkState();
}

void EventStatus::getEntryCount()
{
    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();
    auto methodCall = bus.new_method_call(
        "xyz.openbmc_project.Settings", "/xyz/openbmc_project/logging/settings",
        "org.freedesktop.DBus.Properties", "Get");

    methodCall.append("xyz.openbmc_project.Logging.Settings", "ipmiEntryCount");
    try
    {
        auto reply = bus.call(methodCall);

        std::variant<uint16_t> value;
        reply.read(value);
        entryCount = std::get<uint16_t>(value);
    }
    catch (const sdbusplus::exception_t& e)
    {
        std::cerr << "Failed to update ipmiEntryCount " << std::endl;
    }
}

bool getLastSelDelStatus()
{
    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();
    auto methodCall = bus.new_method_call(
        "xyz.openbmc_project.Settings", "/xyz/openbmc_project/logging/settings",
        "org.freedesktop.DBus.Properties", "Get");

    methodCall.append("xyz.openbmc_project.Logging.Settings", "selDelStatus");
    try
    {
        auto reply = bus.call(methodCall);

        std::variant<bool> value;
        reply.read(value);
        return std::get<bool>(value);
    }
    catch (const sdbusplus::exception_t& e)
    {
        std::cerr << "dbus-sensors: Failed to update Sel delete Status "
                  << std::endl;
        return 0;
    }
}

void setLastSelDelStatus()
{
    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();
    auto methodCall = bus.new_method_call(
        "xyz.openbmc_project.Settings", "/xyz/openbmc_project/logging/settings",
        "org.freedesktop.DBus.Properties", "Set");

    std::variant<bool> value = false;
    methodCall.append("xyz.openbmc_project.Logging.Settings", "selDelStatus",
                      value);
    try
    {
        bus.call(methodCall);
    }
    catch (const sdbusplus::exception_t& e)
    {
        std::cerr << "Failed to update Sel delete Status " << std::endl;
    }
}

bool getSELPolicy()
{
    std::string policyStr;
    static constexpr const char* loggingSettingIntf =
        "xyz.openbmc_project.Logging.Settings";
    static constexpr const char* loggingSettingObjPath =
        "/xyz/openbmc_project/logging/settings";

    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();
    auto methodCall = bus.new_method_call(
        "xyz.openbmc_project.Settings", loggingSettingObjPath,
        "org.freedesktop.DBus.Properties", "Get");
    methodCall.append(loggingSettingIntf, "SelPolicy");

    try
    {
        auto reply = bus.call(methodCall);

        std::variant<std::string> value;
        reply.read(value);
        policyStr = std::get<std::string>(value);
    }
    catch (const sdbusplus::exception_t& e)
    {
        std::cerr << "Failed to get Selpolicy " << std::endl;
    }

    if (policyStr == "xyz.openbmc_project.Logging.Settings.Policy.Linear")
    {
        return false;
    }
    else
        return true;
}

void EventStatus::checkState()
{
    uint16_t currentState = this->state;
    uint16_t newState = [&]() {
        if (getLastSelDelStatus())
        {
            setLastSelDelStatus();
            return static_cast<uint16_t>(logOffset::none);
        }
        if (!entryCount)
        {
            return static_cast<uint16_t>(logOffset::Cleared);
        }
        if (entryCount == 1 &&
            currentState == static_cast<uint16_t>(logOffset::Cleared))
        {
            return static_cast<uint16_t>(logOffset::Cleared);
        }
        if (getSELPolicy())
        {
            return static_cast<uint16_t>(logOffset::none);
        }
        else if (entryCount >= (maxEntries - 1))
        {
            return static_cast<uint16_t>(logOffset::Full);
        }
        else if ((entryCount >= (maxEntries * 0.7)))
        {
            return static_cast<uint16_t>(logOffset::AlmostFull);
        }
        return static_cast<uint16_t>(logOffset::none);
    }();

    if (currentState != newState)
    {
        updateState(sensorInterface, newState);

        std::vector<std::string> logData;
        std::vector<uint8_t> EventDatas;
        std::string eventName;
        auto findEventDetails = EventMapper.find(newState);
        if (findEventDetails == EventMapper.end())
        {
            return; // no event to log
        }

        auto resp = findEventDetails->second;
        eventName = resp.first;
        EventDatas = resp.second;
        logData.push_back(this->name);
        logData.push_back(eventName);
        logData.push_back(objectPath);
        addSelEntry(conn, logData, EventDatas, true);
    }
}

void EventStatus::incrementCount()
{
    entryCount++;
    checkState();
}

void EventStatus::decrementCount()
{
    boost::asio::io_context io;
    static boost::asio::steady_timer timer(io);
    timer.expires_after(std::chrono::seconds(1));
    entryCount--;
    if (!entryCount)
    {
        checkState();
    }
}

void EventStatus::setupRead(std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    std::vector<std::string> objectPaths;
    auto mapperCall = conn->new_method_call(
        "xyz.openbmc_project.ObjectMapper",
        "/xyz/openbmc_project/object_mapper",
        "xyz.openbmc_project.ObjectMapper", "GetSubTreePaths");
    mapperCall.append("/xyz/openbmc_project/logging/ipmi");
    mapperCall.append(2);
    mapperCall.append(
        std::vector<std::string>({"xyz.openbmc_project.Logging.Entry"}));

    try
    {
        auto reply = conn->call(mapperCall);
        reply.read(objectPaths);
    }
    catch (const sdbusplus::exception_t& e)
    {
        objectPaths.clear();
    }
    entryCount = static_cast<uint16_t>(objectPaths.size());
}

EventStatus::~EventStatus()
{
    objServer.remove_interface(sensorInterface);
}
