/*
// Copyright (c) 2022 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "DBusADCSensor.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <sdbusplus/asio/property.hpp>

#include <variant>

static constexpr double roundFactor = 10000; // 3 decimal places
static constexpr const char* sensorObjectType = "DBusADC";

DBusADCSensor::DBusADCSensor(
    sdbusplus::asio::object_server& ObjectServer,
    std::shared_ptr<sdbusplus::asio::connection>& DbusConnection,
    boost::asio::io_context& Io, const std::string& SensorName,
    std::vector<thresholds::Threshold>&& Thresholds,
    const std::string& DbusAdcService, const std::string& DbusAdcObjPath,
    const std::string& DbusAdcIface, const std::string& DbusAdcProperty,
    const double Vref, const unsigned ResolutionBits, const double ScaleFactor,
    const float PollRate, PowerState ReadState,
    const std::string& SensorConfiguration,
    std::optional<BridgeGpio>&& BridgeGpio) :
    Sensor(escapeName(SensorName), std::move(Thresholds), SensorConfiguration,
           sensorObjectType, false, false, Vref / ScaleFactor, 0,
           DbusConnection, ReadState),
    std::enable_shared_from_this<DBusADCSensor>(), objServer(ObjectServer),
    dbusConn(DbusConnection), waitTimer(Io), dbusAdcService(DbusAdcService),
    dbusAdcObjPath(DbusAdcObjPath), dbusAdcIface(DbusAdcIface),
    dbusAdcProperty(DbusAdcProperty), vref(Vref),
    resolutionBits(ResolutionBits), scaleFactor(ScaleFactor),
    sensorPollMs(static_cast<unsigned int>(PollRate * 1000)),
    bridgeGpio(std::move(BridgeGpio)), thresholdTimer(Io)
{
    if (bridgeGpio)
    {
        bridgeGpio->initialize();
    }

    sensorInterface =
        objServer.add_interface("/xyz/openbmc_project/sensors/voltage/" + name,
                                "xyz.openbmc_project.Sensor.Value");
    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objServer.add_interface(
                "/xyz/openbmc_project/sensors/voltage/" + name, interface);
    }
    association = objServer.add_interface(
        "/xyz/openbmc_project/sensors/voltage/" + name, association::interface);
    setInitialProperties(sensor_paths::unitVolts);
}

DBusADCSensor::~DBusADCSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void DBusADCSensor::read()
{
    std::weak_ptr<DBusADCSensor> weakRef = weak_from_this();

    sdbusplus::asio::getProperty<uint16_t>(
        *dbusConnection, dbusAdcService, dbusAdcObjPath, dbusAdcIface,
        dbusAdcProperty,
        [weakRef](const boost::system::error_code ec, uint16_t property) {
            if (std::shared_ptr<DBusADCSensor> self = weakRef.lock())
            {
                self->handleResponse(ec, property);
            }
            else
            {
                std::cerr << "dbusadcsensor read weakref no self\n";
            }
        });
}

void DBusADCSensor::setupRead(void)
{
    if (bridgeGpio)
    {
        bridgeGpio->set(1);

        std::weak_ptr<DBusADCSensor> weakRef = weak_from_this();

        waitTimer.expires_from_now(
            boost::posix_time::milliseconds(bridgeGpio->getSetupTimeMs()));
        waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                std::cerr << "dbusadcsensor bridge stable read cancelled\n";
                return;
            }

            if (std::shared_ptr<DBusADCSensor> self = weakRef.lock())
            {
                self->read();
            }
        });
    }
    else
    {
        read();
    }
}

void DBusADCSensor::handleResponse(const boost::system::error_code& err,
                                   uint16_t rawData)
{
    if (!err)
    {
        try
        {
            double value = std::round((static_cast<double>(rawData) * vref /
                                       (1 << resolutionBits) / scaleFactor) *
                                      roundFactor) /
                           roundFactor;

            updateValue(value);
        }
        catch (const std::invalid_argument&)
        {
            incrementError();
        }
    }
    else
    {
        incrementError();
    }

    if (bridgeGpio)
    {
        bridgeGpio->set(0);
    }

    std::weak_ptr<DBusADCSensor> weakRef = weak_from_this();

    waitTimer.expires_from_now(boost::posix_time::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            std::cerr << "dbusadcsensor scheduled poll cancelled\n";
            return;
        }

        if (std::shared_ptr<DBusADCSensor> self = weakRef.lock())
        {
            self->setupRead();
        }
        else
        {
            std::cerr << "dbusadcsensor scheduled poll weakref no self\n";
        }
    });
}

void DBusADCSensor::checkThresholds(void)
{
    if (!readingStateGood())
    {
        return;
    }

    thresholds::checkThresholdsPowerDelay(weak_from_this(), thresholdTimer);
}
