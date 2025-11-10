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

class OSStatus : public Discrete, public std::enable_shared_from_this<OSStatus>
{
  public:
    OSStatus(sdbusplus::asio::object_server& objectServer,
             std::shared_ptr<sdbusplus::asio::connection>& conn,
             const std::string& sensorName,
             const std::string& sensorConfiguration);
    ~OSStatus() override;

    std::string status;

  private:
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::bus::match::match> osEventMatcher;
};
