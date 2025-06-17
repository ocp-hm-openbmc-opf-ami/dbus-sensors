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
    checkState();
}

void EventStatus::checkState()
{
    uint16_t currentState = this->state;
    uint16_t newState = [&]() {
        if (!entryCount)
        {
            return static_cast<uint16_t>(logOffset::Cleared);
        }
        else if ((entryCount >= (maxEntries * 0.7)) &&
                 (entryCount < (maxEntries - 1)))
        {
            return static_cast<uint16_t>(logOffset::AlmostFull);
        }
        else if (entryCount >= (maxEntries - 1))
        {
            return static_cast<uint16_t>(logOffset::Full);
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
    checkState();
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
