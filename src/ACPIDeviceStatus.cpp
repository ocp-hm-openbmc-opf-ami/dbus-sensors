#include <unistd.h>

#include <ACPIDeviceStatus.hpp>

#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

ACPIDeviceStatus::ACPIDeviceStatus(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    const std::string& deviceName, std::optional<uint8_t> deviceBus,
    std::optional<uint8_t> deviceAddress,
    const std::string& sensorConfiguration) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objServer(objectServer), waitTimer(io), deviceName(deviceName),
    deviceBus(deviceBus), deviceAddress(deviceAddress), conn(conn)
{

    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/acpidevice/" + name,
        "xyz.openbmc_project.Sensor.State");

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/acpidevice/" + name,
        association::interface);
    setInitialProperties();
}

ACPIDeviceStatus::~ACPIDeviceStatus()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void ACPIDeviceStatus::setupRead(void)
{

    deviceFunction = {
        {"PowerSupply", [this]() { psuMonitorState(); }},
    };

    auto it = deviceFunction.find(deviceName);
    if (it != deviceFunction.end())
    {
        it->second();
    }
}

// Function to search for a file recursively
fs::path ACPIDeviceStatus::findFile(const fs::path& directory,
                                    const std::string& filename)
{
    for (const auto& entry : fs::recursive_directory_iterator(directory))
    {
        if (entry.is_regular_file() && entry.path().filename() == filename)
        {
            return entry.path();
        }
    }
    return ""; // Return an empty path if the file is not found
}

void ACPIDeviceStatus::psuMonitorState()
{

    const std::string objPath =
        "/xyz/openbmc_project/sensors/acpidevice/" + name;
    std::vector<std::string> logData(logDataMaxSize);
    std::vector<uint8_t> eventData(selEvtDataMaxSize, 0xFF);
    std::pair<uint8_t, bool> offsetAndEvent;
    uint8_t oldValue = reading;
    logData[0] = name;
    logData[2] = objPath;
    logData[3] = "SensorDeviceACPIPowerStateAssert";

    reading = 0;
    if (!(deviceAddress.has_value()) || !(deviceAddress.has_value()))
    {
        std::cerr << "i2c bus address not configured \n";
        logData[1] = "D3";
        eventData[0] = static_cast<uint8_t>(ACPI::D3);
        addSelEntry(conn, logData, eventData, true);
        updateState(sensorInterface, reading);
        return;
    }

    std::ostringstream hex;
    hex << std::hex << static_cast<int>(deviceAddress.value());
    const std::string& addrHexStr = hex.str();
    int data;
    fs::path hwmonPath = "/sys/bus/i2c/devices/" +
                         std::to_string(deviceBus.value()) + "-00" + addrHexStr;
    if (fs::exists(hwmonPath) && fs::is_directory(hwmonPath))
    {
        std::string targetFilename = "power1_input";
        fs::path filePath = findFile(hwmonPath, targetFilename);
        if (filePath.empty())
        {
            std::cerr << "filePath not found \n";
            return; // temptest
        }
        std::optional<std::string> val = openAndRead(filePath);
        if (val.has_value())
        {
            try
            {
                std::cerr << filePath << "\n";
                data = std::stoi(val.value());
            }
            catch (const std::invalid_argument& e)
            {
                data = -1;
                std::cerr << "Invalid argument: " << e.what() << std::endl;
            }
            if (data > 0)
            {
                reading = reading | (1 << static_cast<uint8_t>(ACPI::D0));
                logData[1] = "D0";
                eventData[0] = static_cast<uint8_t>(ACPI::D0);
                offsetAndEvent = {static_cast<uint8_t>(ACPI::D0), true};
                if (assertedEvents.insert(offsetAndEvent).second == true)
                {
                    addSelEntry(conn, logData, eventData, true);
                }
                offsetAndEvent = {static_cast<uint8_t>(ACPI::D1), true};
                if (assertedEvents.erase(offsetAndEvent) == 1)
                {
                    logData[1] = "D1";
                    logData[3] = "SensorDeviceACPIPowerStateDeassert";
                    eventData[0] = static_cast<uint8_t>(ACPI::D1);
                    addSelEntry(conn, logData, eventData, false);
                }
            }
            else if (data == 0)
            {
                reading = reading | (1 << static_cast<uint8_t>(ACPI::D1));
                logData[1] = "D1";
                eventData[0] = static_cast<uint8_t>(ACPI::D1);
                offsetAndEvent = {static_cast<uint8_t>(ACPI::D1), true};
                if (assertedEvents.insert(offsetAndEvent).second == true)
                {
                    addSelEntry(conn, logData, eventData, true);
                }
            }
            else
            {
                reading = reading | (1 << static_cast<uint8_t>(ACPI::D3));
                logData[1] = "D3";
                eventData[0] = static_cast<uint8_t>(ACPI::D3);
                offsetAndEvent = {static_cast<uint8_t>(ACPI::D3), true};
                if (assertedEvents.insert(offsetAndEvent).second == true)
                {
                    addSelEntry(conn, logData, eventData, true);
                }

                offsetAndEvent = {static_cast<uint8_t>(ACPI::D1), true};
                if (assertedEvents.erase(offsetAndEvent) == 1)
                {
                    logData[1] = "D1";
                    logData[3] = "SensorDeviceACPIPowerStateDeassert";
                    eventData[0] = static_cast<uint8_t>(ACPI::D1);
                    addSelEntry(conn, logData, eventData, false);
                }
            }
        }
    }
    if (oldValue != reading)
    {
        updateState(sensorInterface, reading);
    }
    restartRead();
}

void ACPIDeviceStatus::restartRead()
{
    std::weak_ptr<ACPIDeviceStatus> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        std::shared_ptr<ACPIDeviceStatus> self = weakRef.lock();
        if (!self)
        {
            return;
        }
        self->setupRead();
    });
}
