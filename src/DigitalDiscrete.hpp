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

enum class DigitalType : unsigned int
{
    State = 0x03,
    Failure,
    Limit,
    Performance,
    Presence = 0x08,
    Enabled,
};

namespace fs = std::filesystem;

class DigitalDiscrete :
    public Discrete,
    public std::enable_shared_from_this<DigitalDiscrete>
{
  public:
    DigitalDiscrete(sdbusplus::asio::object_server& objectServer,
                    std::shared_ptr<sdbusplus::asio::connection>& conn,
                    boost::asio::io_context& io, const std::string& sensorName,
                    const std::string& sensorConfiguration,
                    unsigned int& eventType, unsigned int& subType);
    ~DigitalDiscrete() override;

    static constexpr size_t selEvtDataMaxSize = 3;
    std::string baseObj = "/xyz/openbmc_project/sensors/chassisstate/";
    void setupRead(void);

  private:
    sdbusplus::asio::object_server& objServer;
    unsigned int eveType;
    unsigned int subType;
    std::map<unsigned int, std::function<void(void)>> deviceFunction;
    std::shared_ptr<sdbusplus::bus::match_t> powerMonitor;
    void monitorState(void);
    void monitorFailure(void);
    void monitorLimit(void);
    void monitorPerformance(void);
    void monitorPresence(void);
    void monitorEnabled(void);

    const static constexpr char* powerInterface =
        "xyz.openbmc_project.State.Host";
    const static constexpr char* powerPath = "/xyz/openbmc_project/state/host0";
    const static constexpr char* powerProperty = "CurrentHostState";
};
