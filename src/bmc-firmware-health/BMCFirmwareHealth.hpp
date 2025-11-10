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
    sensorFailure = 4,
    fruFailure = 5
};

class BMCFirmwareHealth :
    public Discrete,
    public std::enable_shared_from_this<BMCFirmwareHealth>
{
  public:
    BMCFirmwareHealth(sdbusplus::asio::object_server& objectServer,
                      std::shared_ptr<sdbusplus::asio::connection>& conn,
                      boost::asio::io_context& io,
                      const std::string& sensorName,
                      const std::string& sensorConfiguration);
    ~BMCFirmwareHealth() override;
    void setupRead(void);

  private:
    sdbusplus::asio::object_server& objServer;
    boost::asio::steady_timer waitTimer;
    std::shared_ptr<sdbusplus::asio::connection>& conn;
    void restartRead();
    void monitorState();
};
