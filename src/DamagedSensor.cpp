
#include "DamagedSensor.hpp"

#include "DeviceMgmt.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <boost/asio/io_context.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <iostream>
#include <string>
#include <system_error>
#include <vector>

static constexpr double maxValueTemperature = 127;  // DegreesC
static constexpr double minValueTemperature = -128; // DegreesC

DamagedSensor::DamagedSensor([[maybe_unused]] const std::string& path,
                             const std::string& objectType,
                             sdbusplus::asio::object_server& objectServer,
                             std::shared_ptr<sdbusplus::asio::connection>& conn,
                             [[maybe_unused]] boost::asio::io_context& io,
                             const std::string& sensorName,
                             std::vector<thresholds::Threshold>&& thresholdsIn,
                             const std::string& sensorConfiguration) :
    Sensor(boost::replace_all_copy(sensorName, " ", "_"),
           std::move(thresholdsIn), sensorConfiguration, objectType, false,
           false, maxValueTemperature, minValueTemperature, conn),
    objServer(objectServer), path(path)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        "xyz.openbmc_project.Sensor.Value");

    for (const auto& threshold : thresholds)
    {
        std::string interface = thresholds::getInterface(threshold.level);
        thresholdInterfaces[static_cast<size_t>(threshold.level)] =
            objectServer.add_interface(
                "/xyz/openbmc_project/sensors/temperature/" + name, interface);
    }
    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/temperature/" + name,
        association::interface);
    setInitialProperties(sensor_paths::unitDegreesC);
}

DamagedSensor::~DamagedSensor()
{
    for (const auto& iface : thresholdInterfaces)
    {
        objServer.remove_interface(iface);
    }
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void DamagedSensor::setupRead()
{
    if (!readingStateGood())
    {
        markAvailable(false);
        updateValue(std::numeric_limits<double>::quiet_NaN());
        return;
    }
}

void DamagedSensor::checkThresholds()
{
    thresholds::checkThresholds(this);
}
