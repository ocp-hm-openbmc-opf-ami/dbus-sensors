#include <unistd.h>

#include <WatchdogSensor.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

WatchdogSensor::WatchdogSensor(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    /*boost::asio::io_service& io,*/
    const std::string& sensorName, const std::string& sensorConfiguration) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objServer(objectServer)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/watchdog/" + name,
        "xyz.openbmc_project.Sensor.State");

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/watchdog/" + name,
        association::interface);
    setInitialProperties();

    const std::string objPath = "/xyz/openbmc_project/sensors/watchdog/" + name;
    auto watchdogEventMatcherCallback = [this, &conn, objPath](
                                            sdbusplus::message::message& msg) {
        std::optional<std::string_view> expireAction;

        // SEL event data is three bytes where 0xFF means
        // unspecifiedselEvtDataMaxSize
        std::vector<uint8_t> eventData(selEvtDataMaxSize, 0xFF);
        std::vector<std::string> logData;

        sdbusplus::message::message getWatchdogStatus =
            conn->new_method_call(msg.get_sender(), msg.get_path(),
                                  "org.freedesktop.DBus.Properties", "GetAll");
        getWatchdogStatus.append("xyz.openbmc_project.State.Watchdog");
        boost::container::flat_map<std::string,
                                   std::variant<std::string, uint64_t, bool>>
            watchdogStatus;

        try
        {
            sdbusplus::message::message getWatchdogStatusResp =
                conn->call(getWatchdogStatus);
            getWatchdogStatusResp.read(watchdogStatus);
        }
        catch (const sdbusplus::exception_t&)
        {
            std::cerr << "error getting watchdog status from " << msg.get_path()
                      << "\n";
            return;
        }

        auto getExpireAction = watchdogStatus.find("ExpireAction");
        if (getExpireAction != watchdogStatus.end())
        {
            expireAction = std::get<std::string>(getExpireAction->second);
            expireAction->remove_prefix(std::min(
                expireAction->find_last_of(".") + 1, expireAction->size()));
        }

        std::string action{*expireAction};
        uint16_t offset;
        auto findEvent = eventType.find(action.c_str());
        if (findEvent != eventType.end())
        {
            offset = static_cast<uint16_t>(findEvent->second);
            updateState(sensorInterface, static_cast<uint16_t>(1 << offset));
            eventData[0] = static_cast<uint8_t>(offset);
        }

        logData.push_back(name);
        logData.push_back(action);
        logData.push_back(objPath);
        logData.push_back("SensorWatchdog2");
        auto wdt_nolog = watchdogStatus.find("LogTimeout");
        if (wdt_nolog != watchdogStatus.end())
        {
            if (std::get<bool>(wdt_nolog->second))
            {
                addSelEntry(conn, logData, eventData, true);
            }
        }
    };

    watchdogEventMatcher = std::make_shared<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='xyz.openbmc_project.Watchdog',"
        "member='Timeout'",
        std::move(watchdogEventMatcherCallback));
}

WatchdogSensor::~WatchdogSensor()
{
    objServer.remove_interface(sensorInterface);
}
