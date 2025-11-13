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

namespace fs = std::filesystem;
static constexpr unsigned int sensorPollMs = 2000;
static constexpr size_t selEvtDataMaxSize = 3;
static constexpr size_t logDataMaxSize = 4;

enum class ACPI
{
    D0,
    D1,
    D2,
    D3,
};

class ACPIDeviceStatus :
    public Discrete,
    public std::enable_shared_from_this<ACPIDeviceStatus>
{
  public:
    ACPIDeviceStatus(sdbusplus::asio::object_server& objectServer,
                     std::shared_ptr<sdbusplus::asio::connection>& conn,
                     boost::asio::io_context& io, const std::string& sensorName,
                     const std::string& deviceName,
                     std::optional<uint8_t> deviceBus,
                     std::optional<uint8_t> deviceAddress,
                     const std::string& sensorConfiguration);
    ~ACPIDeviceStatus() override;
    void setupRead(void);

  private:
    uint8_t reading = 0;
    std::map<std::string, std::function<void(void)>> deviceFunction;
    sdbusplus::asio::object_server& objServer;
    boost::asio::steady_timer waitTimer;
    std::string deviceName;
    std::optional<uint8_t> deviceBus;
    std::optional<uint8_t> deviceAddress;
    std::shared_ptr<sdbusplus::asio::connection>& conn;
    boost::container::flat_set<std::pair<uint8_t, bool>> assertedEvents;
    fs::path findFile(const fs::path& directory, const std::string& filename);
    void restartRead();
    void psuMonitorState();
};
