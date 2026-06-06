#pragma once

#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#define EVENT_SDR_TYPE 3

namespace fs = std::filesystem;

// list of supported Power Unit Events
enum class PowerUnitEvent : uint8_t
{
    powerDown = (1 << 0),
    powerCycle = (1 << 1)
};

class PowerUnit :
    public Discrete,
    public std::enable_shared_from_this<PowerUnit>
{
  public:
    PowerUnit(sdbusplus::asio::object_server& objectServer,
              std::shared_ptr<sdbusplus::asio::connection>& conn,
              boost::asio::io_context& io, const std::string& sensorName,
              const std::string& sensorConfiguration,
              std::optional<uint8_t> sensorSDRType, uint16_t sensorNumber,
              uint8_t lun);
    ~PowerUnit() override;

    static constexpr size_t selEvtDataMaxSize = 3;
    std::string baseObj = "/xyz/openbmc_project/sensors/powerunit/";

  private:
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::bus::match_t> powerCycleMonitor;
    std::shared_ptr<sdbusplus::bus::match_t> powerOffMonitor;

    const static constexpr char* hostInterface =
        "xyz.openbmc_project.State.Host";
    const static constexpr char* hostPath = "/xyz/openbmc_project/state/host0";
    const static constexpr char* chassisPath =
        "/xyz/openbmc_project/state/chassis0";
    const static constexpr char* chassisInterface =
        "xyz.openbmc_project.State.Chassis";
    const static constexpr char* hostProperty = "RequestedHostTransition";
    const static constexpr char* chassisProperty = "CurrentPowerState";
};
