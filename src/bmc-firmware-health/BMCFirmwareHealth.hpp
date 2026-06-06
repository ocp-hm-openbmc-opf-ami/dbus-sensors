#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

static constexpr unsigned int sensorPollMs = 2000;
constexpr const char* senInterface = "xyz.openbmc_project.Sensor.Value";
enum managementSubsystemHealth
{
    sensorUnavailable = 0,
    controllerAccessUnavailable = 1,
    ManagementControllerOffline = 2,
    sensorFailure = 4,
    fruFailure = 5
};

static constexpr size_t selEvtDataMaxSize = 3;
static constexpr size_t logDataMaxSize = 4;

constexpr auto fruService = "xyz.openbmc_project.FruDevice";
constexpr auto fruObjectPath = "/xyz/openbmc_project/FruDevice";
constexpr auto fruIntf = "xyz.openbmc_project.FruDeviceManager";

constexpr auto systemLockObjectPath = "/xyz/openbmc_project/control/systemlock";
constexpr auto systemLockIntf =
    "xyz.openbmc_project.Control.Security.SystemLock";
constexpr auto systemLockProp = "SystemLocked";

class BMCFirmwareHealth :
    public Discrete,
    public std::enable_shared_from_this<BMCFirmwareHealth>
{
  public:
    BMCFirmwareHealth(sdbusplus::asio::object_server& objectServer,
                      std::shared_ptr<sdbusplus::asio::connection>& conn,
                      boost::asio::io_context& io,
                      const std::string& sensorName,
                      const std::string& sensorConfiguration,
                      uint16_t sensorNumber, uint8_t lun);
    ~BMCFirmwareHealth() override;
    void setupRead(void);

  private:
    uint8_t state = 0;
    uint8_t prevState = 0; // Track previous state to detect transitions
    bool isFRUAccessible();
    sdbusplus::asio::object_server& objServer;
    boost::asio::steady_timer waitTimer;
    std::shared_ptr<sdbusplus::asio::connection>& conn;
    void restartRead();
    void monitorState();
};
