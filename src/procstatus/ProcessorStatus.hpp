#pragma once

#include <Discrete.hpp>
#include <Utils.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/streambuf.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
static constexpr unsigned int sensorPollMs = 2000;

static constexpr size_t ProcessorselEvtDataMaxSize = 3;

enum class CpuEvent : uint16_t
{
    IERR = 0x00,
    ThermalTrip = 0x01,
    FRB1 = 0x02,
    FRB2 = 0x03,
    PresenceDetected = 0x07,
    Throttled = 0x0A,
    UncorrectableMachineCheckException = 0x0B
};

// Functions to retrieve logging data
std::vector<uint8_t> getSelEventData(CpuEvent event);
std::string getEventDescription(CpuEvent event);

struct VariantNamesVisitor
{
    std::vector<std::string> operator()(const std::string& value) const
    {
        return {value}; // wrap single string in a vector
    }

    std::vector<std::string> operator()(
        const std::vector<std::string>& vec) const
    {
        return vec;
    }

    template <typename T>
    std::vector<std::string> operator()(const T&) const
    {
        std::cerr << "Unexpected type for GpioName\n";
        return {};
    }
};

class ProcessorStatus :
    public Discrete,
    public std::enable_shared_from_this<ProcessorStatus>
{
  public:
    ProcessorStatus(sdbusplus::asio::object_server& objectServer,
                    std::shared_ptr<sdbusplus::asio::connection>& conn,
                    boost::asio::io_context& io, const std::string& sensorName,
                    const std::vector<std::string>& gpioNames,
                    const std::string& sensorConfiguration,
                    const std::vector<std::string>& dbusPaths,
                    const std::vector<std::string>& dbusIfaces,
                    const std::vector<std::string>& dbusProperties,
                    bool dbusEnabled, uint16_t sensorNumber, uint8_t lun);
    ~ProcessorStatus() override;

    std::string gpio;

  private:
    uint16_t currentState = 0;
    sdbusplus::asio::object_server& objServer;
    // GPIO Lines and Event Descriptors
    std::vector<gpiod::line> gpioLines;
    std::vector<boost::asio::posix::stream_descriptor> gpioEventDescriptors;
    std::vector<CpuEvent> gpioEventTypes; // Maps line index to event type
    std::map<std::string, CpuEvent> gpioEventMap;
    bool dbus;
    bool hasDbusConfig = false;
    bool hasGpioConfig = false;
    boost::asio::steady_timer waitTimer;
    std::shared_ptr<sdbusplus::asio::connection>& conn;
    std::vector<std::string> DBusPaths;
    std::vector<std::string> DBusIfaces;
    std::vector<std::string> DBusProperties;

    bool setupEvent(std::shared_ptr<sdbusplus::asio::connection>& conn,
                    const std::string& procGpioName, gpiod::line& gpioLine,
                    boost::asio::posix::stream_descriptor& gpioEventDescriptor,
                    CpuEvent eventType);

    void monitor(std::shared_ptr<sdbusplus::asio::connection>& conn,
                 boost::asio::posix::stream_descriptor& event,
                 gpiod::line& line, CpuEvent eventType);
    void monitorDbus();
    void pollGpioStates();
    void restartRead();
};
