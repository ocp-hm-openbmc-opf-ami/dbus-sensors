#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
static constexpr size_t selEvtDataMaxSize = 3;
static constexpr size_t logDataMaxSize = 4;

enum class ACPI
{
    S0_G0,
    S1,
    S2,
    S3,
    S4,
    S5_G2,
    S4_S5,
    G3,
    SLEEP,
    G1_SLEEP,
    OVERRIDE,
    LEGACY_ON,
    LEGACY_OFF,
    Unknown,
};

class ACPISystemStatus :
    public Discrete,
    public std::enable_shared_from_this<ACPISystemStatus>
{
  public:
    ACPISystemStatus(sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     const std::string& sensorName,
                     const std::string& sensorConfiguration);
    ~ACPISystemStatus() override;

  private:
    uint16_t reading = 0;
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::bus::match_t> powerMonitor;
    std::shared_ptr<sdbusplus::asio::connection>& conn;
    void monitorState(std::shared_ptr<sdbusplus::asio::connection>& conn);
    void propertyInitialize(std::shared_ptr<sdbusplus::asio::connection>& conn);
};
