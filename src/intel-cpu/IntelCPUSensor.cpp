/*
// Copyright (c) 2018-2021 Intel Corporation
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

#include "IntelCPUSensor.hpp"

#include "SensorPaths.hpp"
#include "Thresholds.hpp"
#include "Utils.hpp"
#include "sensor.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/descriptor_base.hpp>
#include <boost/container/flat_map.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

IntelCPUSensor::IntelCPUSensor(
    const std::string& path, const std::string& objectType,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const std::string& sensorConfiguration, int cpuId, bool show,
    double dtsOffset, const SensorProperties& sensorProperties) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectType, false, false, sensorProperties.max, sensorProperties.min,
           conn, PowerState::on),
    objServer(objectServer), inputDev(io),
    nameTcontrol("Tcontrol CPU" + std::to_string(cpuId)), path(path),
    privTcontrol(std::numeric_limits<double>::quiet_NaN()),
    dtsOffset(dtsOffset), show(show), scaleFactor(sensorProperties.scaleFactor)

{
    if (show)
    {
        std::string interfacePath = sensorProperties.path + name;
        sensorInterface = objectServer.add_interface(
            interfacePath, "xyz.openbmc_project.Sensor.Value");
        for (const auto& threshold : thresholds)
        {
            std::string interface = thresholds::getInterface(threshold.level);
            thresholdInterfaces[static_cast<size_t>(threshold.level)] =
                objectServer.add_interface(interfacePath, interface);
        }
        association =
            objectServer.add_interface(interfacePath, association::interface);
        setInitialProperties(sensorProperties.units);
    }
    // call setup always as not all sensors call setInitialProperties
    setupPowerMatch(conn);
}

// Create a  dummy "not available" IntelCPUSensor
// This is used to indicate a missing required sensor for
// other services like fan control
IntelCPUSensor::IntelCPUSensor(
    const std::string& objectType, sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    std::vector<thresholds::Threshold>&& thresholdsIn,
    const std::string& sensorConfiguration) :
    Sensor(escapeName(sensorName), std::move(thresholdsIn), sensorConfiguration,
           objectType, false, false, 0, 0, conn, PowerState::on),
    objServer(objectServer), inputDev(io),
    privTcontrol(std::numeric_limits<double>::quiet_NaN()), dtsOffset(0),
    show(true), minMaxReadCounter(0)
{
    // assume it is a temperature sensor for now
    // support for other type can be added later
    std::string interfacePath =
        "/xyz/openbmc_project/sensors/temperature/" + name;
    const char* units = sensor_paths::unitDegreesC;
    minValue = -128;
    maxValue = 127;
    sensorInterface = objectServer.add_interface(
        interfacePath, "xyz.openbmc_project.Sensor.Value");

    sensorInterface->register_property("Unit", units);
    sensorInterface->register_property("MaxValue", maxValue);
    sensorInterface->register_property("MinValue", minValue);
    sensorInterface->register_property(
        "Value", value, [&](const double& newValue, double& oldValue) {
            return setSensorValue(newValue, oldValue);
        });
    if (!sensorInterface->initialize())
    {
        std::cerr << "error initializing value interface\n";
    }
    if (!availableInterface)
    {
        availableInterface = std::make_shared<sdbusplus::asio::dbus_interface>(
            conn, sensorInterface->get_object_path(), availableInterfaceName);
        availableInterface->register_property(
            "Available", false, [this](const bool propIn, bool& old) {
                if (propIn == old)
                {
                    return 1;
                }
                old = propIn;
                if (!propIn)
                {
                    updateValue(std::numeric_limits<double>::quiet_NaN());
                }
                return 1;
            });
        availableInterface->initialize();
    }
    if (!operationalInterface)
    {
        operationalInterface =
            std::make_shared<sdbusplus::asio::dbus_interface>(
                conn, sensorInterface->get_object_path(),
                operationalInterfaceName);
        operationalInterface->register_property("Functional", true);
        operationalInterface->initialize();
    }
    // call setup always as not all sensors call setInitialProperties
    setupPowerMatch(conn);
}

IntelCPUSensor::~IntelCPUSensor()
{
    // close the input dev to cancel async operations

    try
    {
        inputDev.close();
    }
    catch (...)
    {
        std::cerr << "Exception during inputDev.close()" << std::endl;
    }

    if (show)
    {
        for (const auto& iface : thresholdInterfaces)
        {
            try
            {
                objServer.remove_interface(iface);
            }
            catch (const boost::system::system_error& e)
            {
                // Handle the error gracefully without throwing
                std::cerr << "Caught exception: " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "Unknown exception caught" << std::endl;
            }
        }

        try
        {
            objServer.remove_interface(sensorInterface);
        }
        catch (const boost::system::system_error& e)
        {
            // Handle the error gracefully without throwing
            std::cerr << "Caught exception: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Unknown exception caught" << std::endl;
        }
        try
        {
            objServer.remove_interface(association);
        }
        catch (const boost::system::system_error& e)
        {
            // Handle the error gracefully without throwing
            std::cerr << "Caught exception: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Unknown exception caught" << std::endl;
        }
        try
        {
            objServer.remove_interface(availableInterface);
        }
        catch (const boost::system::system_error& e)
        {
            // Handle the error gracefully without throwing
            std::cerr << "Caught exception: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Unknown exception caught" << std::endl;
        }
        try
        {
            objServer.remove_interface(operationalInterface);
        }
        catch (const boost::system::system_error& e)
        {
            // Handle the error gracefully without throwing
            std::cerr << "Caught exception: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Unknown exception caught" << std::endl;
        }
    }
}

void IntelCPUSensor::setupRead(boost::asio::yield_context yield)
{
    if (readingStateGood())
    {
        inputDev.close();

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        if (path.empty())
        {
            return;
        }
        fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0)
        {
            lg2::error("'{NAME}' unable to open fd!", "NAME", name);
            return;
        }

        inputDev.assign(fd);
    }
    else
    {
        markAvailable(false);
        updateValue(std::numeric_limits<double>::quiet_NaN());
        return;
    }

    std::weak_ptr<IntelCPUSensor> weakRef = weak_from_this();
    boost::system::error_code ec;
    inputDev.async_wait(boost::asio::posix::descriptor_base::wait_read,
                        yield[ec]);
    std::shared_ptr<IntelCPUSensor> self = weakRef.lock();
    if (self)
    {
        self->handleResponse(ec);
    }
}

void IntelCPUSensor::updateMinMaxValues()
{
    double newMin = std::numeric_limits<double>::quiet_NaN();
    double newMax = std::numeric_limits<double>::quiet_NaN();

    const boost::container::flat_map<
        std::string,
        std::vector<std::tuple<const char*, std::reference_wrapper<double>,
                               const char*, std::reference_wrapper<double>>>>
        map = {
            {
                "cap",
                {
                    std::make_tuple("cap_max", std::ref(maxValue), "MaxValue",
                                    std::ref(newMax)),
                    std::make_tuple("cap_min", std::ref(minValue), "MinValue",
                                    std::ref(newMin)),
                },
            },
        };

    if (auto fileParts = splitFileName(path))
    {
        auto& [fileType, fileNr, fileItem] = *fileParts;
        const auto mapIt = map.find(fileItem);
        if (mapIt != map.cend())
        {
            for (const auto& vectorItem : mapIt->second)
            {
                const auto& [suffix, oldValue, dbusName, newValue] = vectorItem;
                auto attrPath = boost::replace_all_copy(path, fileItem, suffix);
                if (auto tmp = readFile(attrPath, scaleFactor))
                {
                    newValue.get() = *tmp;
                }
                else
                {
                    newValue.get() = std::numeric_limits<double>::quiet_NaN();
                }
            }
            if (std::isfinite(newMin) && std::isfinite(newMax) &&
                (newMin < newMax))
            {
                for (const auto& vectorItem : mapIt->second)
                {
                    auto& [suffix, oldValue, dbusName, newValue] = vectorItem;
                    updateProperty(sensorInterface, oldValue, newValue,
                                   dbusName);
                }
            }
        }
    }
}

void IntelCPUSensor::handleResponse(const boost::system::error_code& err)
{
    if ((err == boost::system::errc::bad_file_descriptor) ||
        (err == boost::asio::error::misc_errors::not_found))
    {
        return; // we're being destroyed
    }
    if (err == boost::system::errc::operation_canceled)
    {
        if (readingStateGood())
        {
            if (!loggedInterfaceDown)
            {
                lg2::error("'{NAME}' interface down!", "NAME", name);
                loggedInterfaceDown = true;
            }
            markFunctional(false);
        }
        return;
    }
    loggedInterfaceDown = false;

    if (err)
    {
        incrementError();
        return;
    }

    static constexpr uint32_t bufLen = 128;
    std::string response;
    response.resize(bufLen);
    int rdLen = 0;

    if (fd >= 0)
    {
        rdLen = pread(fd, response.data(), bufLen, 0);
    }

    if (rdLen > 0)
    {
        try
        {
            rawValue = std::stod(response);
            double nvalue = rawValue / scaleFactor;

            if (show)
            {
                updateValue(nvalue);
            }
            else
            {
                value = nvalue;
            }
            if (minMaxReadCounter++ % 8 == 0)
            {
                updateMinMaxValues();
            }

            double gTcontrol = gCpuSensors[nameTcontrol]
                                   ? gCpuSensors[nameTcontrol]->value
                                   : std::numeric_limits<double>::quiet_NaN();
            if (std::isfinite(gTcontrol) && (gTcontrol != privTcontrol))
            {
                // update thresholds when
                // 1) A different valid Tcontrol value is received
                // 2) New threshold values have been read successfully
                // Note: current thresholds can be empty if hwmon attr was not
                // ready when sensor was first created
                std::vector<thresholds::Threshold> newThresholds;
                if (parseThresholdsFromAttr_CPU(newThresholds, path,
                                                scaleFactor, dtsOffset))
                {
                    if (!std::equal(thresholds.begin(), thresholds.end(),
                                    newThresholds.begin(), newThresholds.end()))
                    {
                        if (!newThresholds.empty())
                        {
                            thresholds = newThresholds;
                            if (show)
                            {
                                thresholds::updateThresholds(this);
                            }
                        }
                        /*std::cout << name << ": Tcontrol changed from "
                                  << privTcontrol << " to " << gTcontrol
                                  << "\n";
                        for (auto& threshold : thresholds)
                        {
                            std::cout << name << ": new threshold value "
                                      << threshold.value << "\n";
                        }*/
                    }
                }
                else
                {
                    lg2::error("Failure to update thresholds for '{NAME}'",
                               "NAME", name);
                }
                privTcontrol = gTcontrol;
            }
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
}

void IntelCPUSensor::checkThresholds()
{
    if (show)
    {
        thresholds::checkThresholds(this);
    }
}
