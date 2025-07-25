#pragma once

#include "Thresholds.hpp"
#include "sensor.hpp"

#include <boost/asio/deadline_timer.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/message.hpp>

#include <iostream>
#include <optional>
#include <source_location>
#include <string>
#include <vector>

class BridgeGpio
{
  public:
    BridgeGpio(const std::string& Name, const int Polarity,
               const float SetupTime) :
        name(Name), polarity(Polarity),
        setupTimeMs(static_cast<unsigned int>(SetupTime * 1000))
    {}

    void initialize()
    {
        line = gpiod::find_line(name);
        if (!line)
        {
            std::cerr << "Error finding gpio: " << name << "\n";
            return;
        }

        try
        {
            line.request(
                {"dbusadcsensor", gpiod::line_request::DIRECTION_OUTPUT,
                 polarity == gpiod::line::ACTIVE_HIGH
                     ? 0
                     : gpiod::line_request::FLAG_ACTIVE_LOW});
        }
        catch (const std::system_error&)
        {
            line.release();
            std::cerr << "Error requesting gpio: " << name << "\n";
        }
    }

    void set(int value)
    {
        if (line)
        {
            try
            {
                line.set_value(value);
            }
            catch (const std::system_error& exc)
            {
                std::cerr << "Error set_value: " << exc.what() << "\n";
            }
        }
    }

    unsigned getSetupTimeMs()
    {
        return setupTimeMs;
    }

    ~BridgeGpio()
    {
        if (line)
        {
            set(0);
            line.release();
        }
    }

  private:
    std::string name;
    int polarity;
    unsigned setupTimeMs;
    gpiod::line line;
};

class DBusADCSensorController :
    public std::enable_shared_from_this<DBusADCSensorController>
{
  public:
    DBusADCSensorController(
        std::shared_ptr<sdbusplus::asio::connection>& DBusConnection,
        const std::string& Name, std::string&& DBusService,
        std::string&& DBusObjectPath, std::string&& DBusIface,
        const double Vref, const unsigned ResolutionBits) :
        name(Name), dbusService(DBusService), dbusObjectPath(DBusObjectPath),
        dbusIface(DBusIface), dbusConnection(DBusConnection), vref(Vref),
        resolutionBits(ResolutionBits) {};

    ~DBusADCSensorController()
    {
        disable();
    }

    bool enable()
    {
        try
        {
            auto setPropMessage = dbusConnection->new_method_call(
                dbusService.c_str(), dbusObjectPath.c_str(),
                "org.freedesktop.DBus.Properties", "Set");
            setPropMessage.append(dbusIface.c_str(), "AdcActive",
                                  std::variant<bool>(true));

            dbusConnection->call(setPropMessage);
            enabled = true;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to activate the controller: " << name
                      << ". Error: " << e.what() << '\n';
        }
        return enabled;
    }

    bool disable()
    {
        try
        {
            auto setPropMessage = dbusConnection->new_method_call(
                dbusService.c_str(), dbusObjectPath.c_str(),
                "org.freedesktop.DBus.Properties", "Set");
            setPropMessage.append(dbusIface.c_str(), "AdcActive",
                                  std::variant<bool>(false));

            dbusConnection->call(setPropMessage);
            enabled = false;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to deactivate the controller: " << name
                      << ". Error: " << e.what() << '\n';
        }
        return !enabled;
    }

    double getVref()
    {
        return vref;
    }
    unsigned getResolution()
    {
        return resolutionBits;
    }
    bool isEnabled()
    {
        return enabled;
    }

    const std::string name;
    const std::string dbusService;
    const std::string dbusObjectPath;
    const std::string dbusIface;

  private:
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection;
    const double vref;
    const unsigned resolutionBits;
    bool enabled = false;
};

class DBusADCSensor :
    public Sensor,
    public std::enable_shared_from_this<DBusADCSensor>
{
  public:
    DBusADCSensor(
        sdbusplus::asio::object_server& ObjectServer,
        std::shared_ptr<sdbusplus::asio::connection>& DbusConnection,
        boost::asio::io_context& Io, const std::string& SensorName,
        std::vector<thresholds::Threshold>&& Thresholds,
        const std::string& DbusAdcService, const std::string& DbusAdcObjPath,
        const std::string& DbusAdcIface, const std::string& DbusAdcProperty,
        const double Vref, const unsigned ResolutionBits,
        const double ScaleFactor, const float PollRate, PowerState ReadState,
        const std::string& SensorConfiguration,
        std::optional<BridgeGpio>&& BridgeGpio);
    ~DBusADCSensor() override;
    void read();
    void setupRead();

  private:
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::asio::connection> dbusConn;
    boost::asio::deadline_timer waitTimer;
    const std::string dbusAdcService;
    const std::string dbusAdcObjPath;
    const std::string dbusAdcIface;
    const std::string dbusAdcProperty;
    const double vref;
    const unsigned resolutionBits;
    const double scaleFactor;
    const unsigned int sensorPollMs;
    std::optional<BridgeGpio> bridgeGpio;
    thresholds::ThresholdTimer thresholdTimer;
    void handleResponse(const boost::system::error_code& err, uint16_t rawData);
    void checkThresholds() override;
};
