#pragma once

#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
struct CmpStr
{
    bool operator()(const char* a, const char* b) const
    {
        return std::strcmp(a, b) < 0;
    }
};

// list of supported Power Supply Events
enum class PsuEvent : uint16_t
{
    psuPresenceDetected = (1 << 0),
    PredictiveFailure = (1 << 2),
    psuInputACLost = (1 << 3),
};

static boost::container::flat_map<std::string, std::vector<std::string>>
    eventMatch{{"PredictiveFailure", {"power1_alarm"}},
               {"ACLost", {"in1_beep"}}};

const static boost::container::flat_map<const char*, PsuEvent, CmpStr>
    eventType{{
        {"ACLost", PsuEvent::psuInputACLost},
        {"PredictiveFailure", PsuEvent::PredictiveFailure},
    }};

class PsuStatus :
    public Discrete,
    public std::enable_shared_from_this<PsuStatus>
{
  public:
    PsuStatus(sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_context& io, const std::string& sensorName,
              const std::string& path,
              boost::container::flat_map<std::string, std::vector<std::string>>
                  eventPathList,
              const std::string& sensorConfiguration);
    ~PsuStatus() override;
    void setupRead(void);
    void restartRead(void);
    void updateEvent(uint16_t, uint16_t);
    void initHwmonPath(const std::string);

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::posix::stream_descriptor inputDev;
    boost::asio::steady_timer waitTimer;
    std::string fsPath;
    boost::container::flat_map<std::string, std::vector<std::string>>
        eventPathList;
};
