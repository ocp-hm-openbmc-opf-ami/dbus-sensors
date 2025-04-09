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

namespace fs = std::filesystem;

enum class logStatusEvent : uint8_t
{
    LogAreadCleared = 0x02,
    LogAreaFull = 0x04,
    LogAreaAlmostFull = 0x05,
};

enum class logOffset : uint8_t
{
    none = 0x00,
    Cleared = 0x04,
    Full = 0x10,
    AlmostFull = 0x20,
};

const static boost::container::flat_map<
    uint8_t, std::pair<std::string, std::vector<uint8_t>>>
    EventMapper = {
        {0x04,
         {"Log Area Cleared ",
          {static_cast<uint8_t>(logStatusEvent::LogAreadCleared), 0xff, 0xff}}},
        {0x10,
         {"Log Area Full",
          {static_cast<uint8_t>(logStatusEvent::LogAreaFull), 0xff, 0xff}}},
        {0x20,
         {"Log Area Almost Full",
          {static_cast<uint8_t>(logStatusEvent::LogAreaAlmostFull), 0xff,
           0xff}}},
};

class EventStatus :
    public Discrete,
    public std::enable_shared_from_this<EventStatus>
{
  public:
    EventStatus(sdbusplus::asio::object_server& objectServer,
                std::shared_ptr<sdbusplus::asio::connection>& conn,
                const std::string& sensorName,
                const std::string& sensorConfiguration,
                const uint16_t& testLimit);
    ~EventStatus() override;

    void setupRead(std::shared_ptr<sdbusplus::asio::connection>&);
    void decrementCount();
    void incrementCount();
    void checkState();

  private:
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    uint16_t maxEntries;
    uint16_t entryCount;
    std::string objectPath;
};
