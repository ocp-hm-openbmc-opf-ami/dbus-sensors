#include <unistd.h>

#include <PsuStatus.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

PsuStatus::PsuStatus(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io __attribute__((unused)),
    const std::string& sensorName, const uint64_t bus, const uint64_t address,
    boost::container::flat_map<std::string, std::vector<std::string>> pathList,
    const std::string& sensorConfiguration, uint16_t sensorNumber,
    uint8_t lun) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn, sensorNumber,
             lun),
    conn(conn), objServer(objectServer), inputDev(io), waitTimer(io), bus(bus),
    address(address), eventPathList(pathList)
{
    if (!conn)
    {
        throw std::invalid_argument("D-Bus connection is null");
    }

    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/powersupply/" + name,
        "xyz.openbmc_project.Sensor.State");

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/powersupply/" + name,
        association::interface);

    setInitialProperties();
}

// Function to search for a file recursively
fs::path PsuStatus::findFile(const fs::path& directory,
                             const std::string& filename)
{
    fs::path foundPath;
    uint8_t depth = 0;
    for (const auto& entry : fs::recursive_directory_iterator(directory))
    {
        if (entry.is_directory())
        {
            depth++;
            if (depth == 1)
            {
                foundPath = entry.path();
            }
        }
        else if (entry.is_regular_file() && entry.path().filename() == filename)
        {
            std::string match;
            std::ifstream file(entry.path());
            if (file.is_open())
            {                  // always check whether the file is open
                file >> match; // pipe file's content into stream
                if ((match.compare("pmbus")) == 0)
                {
                    file.close();
                    return foundPath;
                }
                file.close();
            }
        }
    }
    return ""; // Return an empty path if the file is not found
}

void PsuStatus::initHwmonPath(const uint64_t bus, const uint64_t address)
{
    eventPathList.clear();
    fs::path fsPath;
    std::ostringstream hex;
    hex << std::hex << static_cast<uint64_t>(address);
    const std::string& addrHexStr = hex.str();
    fs::path hwmonPath = "/sys/bus/i2c/devices/" + std::to_string(bus) + "-00" +
                         addrHexStr + "/hwmon/";
    if (fs::exists(hwmonPath) && fs::is_directory(hwmonPath))
    {
        std::string targetFilename = "name";
        fsPath = findFile(hwmonPath, targetFilename);
        if (fsPath.empty())
        {
            std::vector<std::string> logData = {
                name, "Configuration error",
                "/xyz/openbmc_project/sensors/powersupply/" + name,
                "PsuStatus"};
            std::vector<uint8_t> eventData = {0x06, 0x00, 0x00};
            addSelEntry(conn, logData, eventData, true, sensorNumber);
            updateState(sensorInterface,
                        (static_cast<uint16_t>(PsuEvent::psuConfigurationErr)));
            std::cerr << "filePath not found \n";
        }
        else
        {
            if (!presenceLogged)
            {
                std::string baseObj =
                    "/xyz/openbmc_project/sensors/powersupply/";
                std::vector<std::string> logData = {
                    name, "Presence detected", baseObj + name, "PsuStatus"};
                std::vector<uint8_t> eventData = {0x00, 0x00, 0x00};

                addSelEntry(conn, logData, eventData, true, sensorNumber);
                presenceLogged = true;
            }
            updateState(sensorInterface,
                        static_cast<uint8_t>(PsuEvent::psuPresenceDetected));
        }
    }

    for (const auto& match : eventMatch)
    {
        const std::vector<std::string>& eventAttrs = match.second;
        const std::string& eventName = match.first;
        for (const auto& eventAttr : eventAttrs)
        {
            std::string eventPath = fsPath;
            eventPath += "/";
            eventPath += eventAttr;
            std::ifstream eventFile(eventPath);
            if (!eventFile.good())
            {
                continue;
            }
            eventPathList[eventName].push_back(eventPath);
        }
    }
}
void PsuStatus::setupRead()
{
    initHwmonPath(bus, address);
    bool ioError = false;
    for (const auto& match : eventPathList)
    {
        const std::vector<std::string>& eventAttrs = match.second;
        for (const auto& eventAttr : eventAttrs)
        {
            int value = 0;
            uint8_t offset = 0;
            std::string hwmonPath = eventAttr;
            std::ifstream stream(hwmonPath);
            if (!stream.good())
            {
                continue;
            }
            std::string line;
            if (!std::getline(stream, line))
            {
                std::cerr << "Error reading status at " << hwmonPath << "\n";
                if (stream.bad() || stream.fail())
                {
                    ioError = true;
                }
            }
            else
            {
                try
                {
                    value = std::stoi(line);
                }
                catch (sdbusplus::exception_t& e)
                {
                    if (debug)
                    {
                        phosphor::logging::log<phosphor::logging::level::ERR>(
                            "Failed to fetch",
                            phosphor::logging::entry("EXCEPTION=%s", e.what()));
                    }
                }
            }

            std::string strr = match.first;
            auto findEvent = eventType.find(strr.c_str());
            if (findEvent == eventType.end())
            {
                continue;
            }

            offset = static_cast<uint8_t>(findEvent->second);

            bool& lastValue = lastEventValues[strr];

            if (value && !lastValue)
            {
                std::vector<std::string> logData = {
                    name, strr,
                    "/xyz/openbmc_project/sensors/powersupply/" + name,
                    "PsuStatus"};

                std::vector<uint8_t> eventData = {offset, 0x00, 0x00};

                addSelEntry(conn, logData, eventData, true, sensorNumber);
            }
            lastValue = value;
            updateEvent(offset, value);
            restartRead();
        }
    }

    if (ioError)
    {
        // File read error (EIO): PSU removed but hwmon sysfs still exists.
        // Clear all state bits so no stale events remain.
        updateState(sensorInterface, 0);
        presenceLogged = false;
    }

    restartRead();
}
void PsuStatus::updateEvent(uint8_t offset, uint8_t value)
{
    uint8_t newValue = state;
    if (value)
    {
        newValue |= offset;
    }
    else if (value == 0)
    {
        newValue &= (~offset);
    }
    if (newValue != state)
    {
        updateState(sensorInterface, (static_cast<uint8_t>(newValue)));
    }
}
void PsuStatus::restartRead(void)
{
    static constexpr double defaultSensorPoll = 1.0;

    static constexpr unsigned int defaultSensorPollMs =
        static_cast<unsigned int>(defaultSensorPoll * 1000);

    unsigned int sensorPollMs = defaultSensorPollMs;
    std::weak_ptr<PsuStatus> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return;
        }
        std::shared_ptr<PsuStatus> self = weakRef.lock();
        if (self)
        {
            self->setupRead();
        }
    });
}

PsuStatus::~PsuStatus()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}
