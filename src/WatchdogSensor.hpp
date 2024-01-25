#pragma once

#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <xyz/openbmc_project/Association/Definitions/server.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

enum class watchdogEvents : uint8_t
{
    noAction = 0x00,
    hardReset = 0x01,
    powerDown = 0x02,
    powerCycle = 0x03,
};


static constexpr size_t selEvtDataMaxSize = 3;

namespace fs = std::filesystem;

struct CmpStr
{
    bool operator()(const char* a, const char* b) const
    {
        return std::strcmp(a, b) < 0;
    }
};

const static boost::container::flat_map<const char*, watchdogEvents, CmpStr>
    eventType{{
        {"None", watchdogEvents::noAction},
        {"HardReset", watchdogEvents::hardReset},
        {"PowerOff", watchdogEvents::powerDown},
        {"PowerCycle", watchdogEvents::powerCycle},
    }};


class WatchdogSensor :
    public Discrete,
    public std::enable_shared_from_this<WatchdogSensor>
{
  public:
    WatchdogSensor(
        sdbusplus::asio::object_server& objectServer,
        std::shared_ptr<sdbusplus::asio::connection>& conn,
        /*boost::asio::io_service& io,*/ const std::string& sensorName,
        const std::string& sensorConfiguration);
    ~WatchdogSensor() override;

    std::string status;

  private:
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::bus::match::match> watchdogEventMatcher;
};
