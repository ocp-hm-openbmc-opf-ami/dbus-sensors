#pragma once

#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

class PowerUnit :
    public Discrete,
    public std::enable_shared_from_this<PowerUnit>
{
  public:
    PowerUnit(sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_context& io, const std::string& sensorName,
              const std::string& sensorConfiguration);
    ~PowerUnit() override;

    static constexpr size_t selEvtDataMaxSize = 3;
    std::string baseObj = "/xyz/openbmc_project/sensors/powerunit/";

  private:
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::bus::match_t> powerMonitor;

    const static constexpr char* powerInterface =
        "xyz.openbmc_project.State.Host";
    const static constexpr char* powerPath = "/xyz/openbmc_project/state/host0";
    const static constexpr char* powerProperty = "CurrentHostState";
};
