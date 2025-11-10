#pragma once

#include "DeviceMgmt.hpp"
#include "Thresholds.hpp"
#include "sensor.hpp"

#include <boost/asio/random_access_file.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

class DamagedSensor :
    public Sensor,
    public std::enable_shared_from_this<DamagedSensor>
{
  public:
    DamagedSensor(const std::string& path, const std::string& objectType,
                  sdbusplus::asio::object_server& objectServer,
                  std::shared_ptr<sdbusplus::asio::connection>& conn,
                  boost::asio::io_context& io, const std::string& sensorName,
                  std::vector<thresholds::Threshold>&& thresholds,
                  const std::string& sensorConfiguration);
    ~DamagedSensor() override;
    void setupRead();

  private:
    sdbusplus::asio::object_server& objServer;
    std::string path;

    void checkThresholds() override;
};

inline boost::container::flat_map<uint64_t, uint64_t> checkSysfsAttributesExist(
    const ManagedObjectType& sensorConfigs)
{
    boost::container::flat_map<uint64_t, uint64_t> device;
    for (const auto& [path, sensor] : sensorConfigs)
    {
        for (const auto& [name, cfg] : sensor)
        {
            auto busCfg = cfg.find("Bus");
            auto addrCfg = cfg.find("Address");
            if ((busCfg == cfg.end()) || (addrCfg == cfg.end()))
            {
                continue;
            }

            if ((std::get_if<uint64_t>(&busCfg->second) == nullptr) ||
                (std::get_if<uint64_t>(&addrCfg->second) == nullptr))
            {
                std::cerr << path.str << " Bus or Address invalid\n";
                continue;
            }
            uint64_t bus = std::get<uint64_t>(busCfg->second);
            uint64_t address = std::get<uint64_t>(addrCfg->second);
            std::ostringstream hex;
            hex << std::hex << static_cast<uint64_t>(address);
            const std::string& addrHexStr = hex.str();
            fs::path hwmonPath = "/sys/bus/i2c/devices/" + std::to_string(bus) +
                                 "-00" + addrHexStr + "/hwmon/";
            if (!fs::exists(hwmonPath) && !fs::is_directory(hwmonPath))
            {
                device.emplace(bus, address);
            }
        }
    }
    return device;
}
