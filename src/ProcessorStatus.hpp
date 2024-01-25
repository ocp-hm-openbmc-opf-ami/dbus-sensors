#pragma once

#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/streambuf.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

enum class  CpuEvent : uint16_t
{
    PresenceDetected = 0x07,
};

class ProcessorStatus :
    public Discrete,
    public std::enable_shared_from_this<ProcessorStatus>
{
  public:
    ProcessorStatus(sdbusplus::asio::object_server& objectServer,
                    std::shared_ptr<sdbusplus::asio::connection>& conn,
                    boost::asio::io_context& io, const std::string& sensorName,
                    const std::string& gpioName,
                    const std::string& sensorConfiguration);
    ~ProcessorStatus() override;

    std::string gpio;

  private:
    sdbusplus::asio::object_server& objServer;
    // GPIO Lines and Event Descriptors
    gpiod::line procPresentLine;
    boost::asio::posix::stream_descriptor procPresentEvent;
    bool setupEvent(std::shared_ptr<sdbusplus::asio::connection>& conn,
                    const std::string& gpioName, gpiod::line& gpioLine,
                    boost::asio::posix::stream_descriptor& gpioEventDescriptor);
    void monitor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                 const std::vector<std::string>& logData,
                 const std::vector<uint8_t> procPresence,
                 boost::asio::posix::stream_descriptor& event,
                 gpiod::line& line);
};
