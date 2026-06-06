#pragma once

#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <cstddef>
#include <memory>
#include <string>

class ChassisIntrusionSensor
{
  public:
    explicit ChassisIntrusionSensor(bool autoRearm,
                                    sdbusplus::asio::object_server& objServer,
                                    uint16_t sensorNumber, uint8_t lun);

    virtual ~ChassisIntrusionSensor();

    void start();

  protected:
    virtual int readSensor() = 0;
    virtual void pollSensorStatus() = 0;
    void updateValue(const size_t& value);

  private:
    std::string mValue;
    // If this sensor uses automatic rearm method. Otherwise, manually rearm it
    bool mAutoRearm;
    std::shared_ptr<sdbusplus::asio::dbus_interface> mIface;
    sdbusplus::asio::object_server& mObjServer;
    uint16_t mSensorNumber;
    uint8_t mLun;
    bool mOverridenState = false;
    bool mInternalSet = false;
    bool mRearmFlag = false;

    int setSensorValue(const std::string& req, std::string& propertyValue);
};

class ChassisIntrusionPchSensor :
    public ChassisIntrusionSensor,
    public std::enable_shared_from_this<ChassisIntrusionPchSensor>
{
  public:
    ChassisIntrusionPchSensor(bool autoRearm, boost::asio::io_context& io,
                              sdbusplus::asio::object_server& objServer,
                              int busId, int slaveAddr, uint16_t sensorNumber,
                              uint8_t lun);

    ~ChassisIntrusionPchSensor() override;

  private:
    int mBusFd{-1};
    int mSlaveAddr{-1};
    boost::asio::steady_timer mPollTimer;
    int readSensor() override;
    void pollSensorStatus() override;
};

class ChassisIntrusionGpioSensor :
    public ChassisIntrusionSensor,
    public std::enable_shared_from_this<ChassisIntrusionGpioSensor>
{
  public:
    ChassisIntrusionGpioSensor(bool autoRearm, boost::asio::io_context& io,
                               sdbusplus::asio::object_server& objServer,
                               bool gpioInverted, uint16_t sensorNumber,
                               uint8_t lun);

    ~ChassisIntrusionGpioSensor() override;

  private:
    bool mGpioInverted{false};
    std::string mPinName = "CHASSIS_INTRUSION";
    gpiod::line mGpioLine;
    boost::asio::posix::stream_descriptor mGpioFd;
    int readSensor() override;
    void pollSensorStatus() override;
};

class ChassisIntrusionHwmonSensor :
    public ChassisIntrusionSensor,
    public std::enable_shared_from_this<ChassisIntrusionHwmonSensor>
{
  public:
    ChassisIntrusionHwmonSensor(bool autoRearm, boost::asio::io_context& io,
                                sdbusplus::asio::object_server& objServer,
                                std::string hwmonName, uint16_t sensorNumber,
                                uint8_t lun);

    ~ChassisIntrusionHwmonSensor() override;

  private:
    std::string mHwmonName;
    std::string mHwmonPath;
    boost::asio::steady_timer mPollTimer;
    int readSensor() override;
    void pollSensorStatus() override;
};
