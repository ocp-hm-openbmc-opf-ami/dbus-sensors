/*
// Copyright (c) 2018 Intel Corporation
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
#include "Utils.hpp"
#include "VariantVisitors.hpp"

#include <fcntl.h>
#include <libudev.h>
#include <peci.h>

#include <boost/algorithm/string/replace.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

// clang-format off
// this needs to be included last or we'll have build issues
#include <linux/peci-ioctl.h>
#if !defined(PECI_MBX_INDEX_DDR_DIMM_TEMP)
#define PECI_MBX_INDEX_DDR_DIMM_TEMP MBX_INDEX_DDR_DIMM_TEMP
#endif
// clang-format on

static constexpr bool debug = false;
static std::unique_ptr<boost::asio::steady_timer> waitTimer = nullptr;
static bool sensorMapUpdated = false;

static constexpr size_t sensorPollLoopMs = 1000;
static constexpr size_t sensorEmptyWaitMs = 10000;
static constexpr size_t failSensorLimit =
    5; // peci timeout is 700ms, 5 timeout would be 3.5seconds
static constexpr size_t sensorFailThrottleMs = 3000;

static constexpr size_t fastPingSeconds = 1;
static constexpr size_t slowPingSeconds = 5;
static constexpr size_t failPingSeconds = 15;

boost::container::flat_map<std::string, std::shared_ptr<IntelCPUSensor>>
    gCpuSensors;
boost::container::flat_map<std::string,
                           std::shared_ptr<sdbusplus::asio::dbus_interface>>
    inventoryIfaces;

enum State
{
    OFF,  // host powered down
    ON,   // host powered on
    READY // host powered on and mem test passed - fully ready
};

struct CPUConfig
{
    CPUConfig(const uint64_t& bus, const uint64_t& addr,
              const std::string& name, const std::string& busName,
              const State& state) :
        bus(bus),
        addr(addr), name(name), busName(busName), state(state)
    {}
    int bus;
    int addr;
    std::string name;
    std::string busName;
    State state;

    bool operator<(const CPUConfig& rhs) const
    {
        // NOLINTNEXTLINE
        return (name < rhs.name);
    }
};

static constexpr const char* peciDev = "/dev/peci-";
static constexpr const char* peciDevPath = "/sys/bus/peci/devices/";
static constexpr const char* rescanPath = "/sys/bus/peci/rescan";
static constexpr const unsigned int rankNumMax = 8;

namespace fs = std::filesystem;

static constexpr auto sensorTypes{std::to_array<const char*>({"XeonCPU"})};
static constexpr auto hiddenProps{std::to_array<const char*>(
    {IntelCPUSensor::labelTcontrol, "Tthrottle", "Tjmax"})};

static const boost::container::flat_map<std::string, SensorProperties>
    sensorPropertiesMap = {
        {"power",
         {"/xyz/openbmc_project/sensors/power/", sensor_paths::unitWatts, 4500,
          0, 1000}},
        {"energy",
         {"/xyz/openbmc_project/sensors/energy/", sensor_paths::unitJoules,
          std::numeric_limits<uint32_t>::max() / 1000000, 0.0, 1000000}},
        {"temp",
         {"/xyz/openbmc_project/sensors/temperature/",
          sensor_paths::unitDegreesC, 127.0, -128.0, 1000}}};

void detectCpuAsync(
    boost::asio::steady_timer& pingTimer, const size_t pingSeconds,
    boost::asio::steady_timer& creationTimer, boost::asio::io_context& io,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    boost::container::flat_set<CPUConfig>& cpuConfigs,
    ManagedObjectType& sensorConfigs);

std::string createSensorName(const std::string& label, const std::string& item,
                             const int& cpuId)
{
    std::string sensorName = label;
    if (item != "input")
    {
        sensorName += " " + item;
    }
    sensorName += " CPU" + std::to_string(cpuId);
    // converting to Upper Camel case whole name
    bool isWordEnd = true;
    std::transform(sensorName.begin(), sensorName.end(), sensorName.begin(),
                   [&isWordEnd](int c) {
        if (std::isspace(c) != 0)
        {
            isWordEnd = true;
        }
        else
        {
            if (isWordEnd)
            {
                isWordEnd = false;
                return std::toupper(c);
            }
        }
        return c;
    });
    return sensorName;
}

bool createSensors(boost::asio::io_context& io,
                   sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
                   boost::container::flat_set<CPUConfig>& cpuConfigs,
                   ManagedObjectType& sensorConfigs)
{
    bool available = false;
    for (const CPUConfig& cpu : cpuConfigs)
    {
        if (cpu.state != State::OFF)
        {
            available = true;
            std::shared_ptr<sdbusplus::asio::dbus_interface>& iface =
                inventoryIfaces[cpu.name];
            if (iface != nullptr)
            {
                continue;
            }
            iface = objectServer.add_interface(
                cpuInventoryPath + std::string("/") + cpu.name,
                "xyz.openbmc_project.Inventory.Item");
            iface->register_property("PrettyName", cpu.name);
            iface->register_property("Present", true);
            iface->initialize();
        }
    }
    if (!available)
    {
        return false;
    }

    if (sensorConfigs.empty())
    {
        return false;
    }

    std::vector<fs::path> hwmonNamePaths;
    findFiles(fs::path(peciDevPath),
              R"(peci-\d+/\d+-.+/peci[-_].+/hwmon/hwmon\d+/name$)",
              hwmonNamePaths, 6);
    if (hwmonNamePaths.empty())
    {
        std::cerr << "No CPU sensors in system\n";
        return false;
    }

    boost::container::flat_set<std::string> scannedDirectories;
    boost::container::flat_set<std::string> createdSensors;

    for (const fs::path& hwmonNamePath : hwmonNamePaths)
    {
        auto hwmonDirectory = hwmonNamePath.parent_path();

        auto ret = scannedDirectories.insert(hwmonDirectory.string());
        if (!ret.second)
        {
            continue; // already searched this path
        }

        fs::path::iterator it = hwmonNamePath.begin();
        std::advance(it, 6); // pick the 6th part for a PECI client device name
        std::string deviceName = *it;

        size_t bus = 0;
        size_t addr = 0;
        if (!getDeviceBusAddr(deviceName, bus, addr))
        {
            continue;
        }

        std::ifstream nameFile(hwmonNamePath);
        if (!nameFile.good())
        {
            std::cerr << "Failure reading " << hwmonNamePath << "\n";
            continue;
        }
        std::string hwmonName;
        std::getline(nameFile, hwmonName);
        nameFile.close();
        if (hwmonName.empty())
        {
            // shouldn't have an empty name file
            continue;
        }
        if (debug)
        {
            std::cout << "Checking: " << hwmonNamePath << ": " << hwmonName
                      << "\n";
        }

        std::string sensorType;
        const SensorData* sensorData = nullptr;
        const std::string* interfacePath = nullptr;
        const SensorBaseConfiguration* baseConfiguration = nullptr;

        for (const auto& [path, cfgData] : sensorConfigs)
        {
            sensorData = &cfgData;
            for (const char* type : sensorTypes)
            {
                sensorType = type;
                auto sensorBase =
                    sensorData->find(configInterfaceName(sensorType));
                if (sensorBase != sensorData->end())
                {
                    baseConfiguration = &(*sensorBase);
                    break;
                }
            }
            if (baseConfiguration == nullptr)
            {
                std::cerr << "error finding base configuration for" << hwmonName
                          << "\n";
                continue;
            }
            auto configurationBus = baseConfiguration->second.find("Bus");
            auto configurationAddress =
                baseConfiguration->second.find("Address");

            if (configurationBus == baseConfiguration->second.end() ||
                configurationAddress == baseConfiguration->second.end())
            {
                std::cerr << "error finding bus or address in configuration";
                continue;
            }

            if (std::get<uint64_t>(configurationBus->second) != bus ||
                std::get<uint64_t>(configurationAddress->second) != addr)
            {
                continue;
            }

            interfacePath = &path.str;
            break;
        }
        if (interfacePath == nullptr)
        {
            std::cerr << "failed to find match for " << hwmonName << "\n";
            continue;
        }

        auto findCpuId = baseConfiguration->second.find("CpuID");
        if (findCpuId == baseConfiguration->second.end())
        {
            std::cerr << "could not determine CPU ID for " << hwmonName << "\n";
            continue;
        }
        int cpuId = std::visit(VariantToUnsignedIntVisitor(),
                               findCpuId->second);

        auto directory = hwmonNamePath.parent_path();
        std::vector<fs::path> inputPaths;
        if (!findFiles(directory,
                       R"((temp|power|energy)\d+_(input|average|cap)$)",
                       inputPaths, 0))
        {
            std::cerr << "No temperature sensors in system\n";
            continue;
        }

        // iterate through all found temp sensors
        for (const auto& inputPath : inputPaths)
        {
            auto fileParts = splitFileName(inputPath);
            if (!fileParts)
            {
                continue;
            }
            auto& [type, nr, item] = *fileParts;
            auto inputPathStr = inputPath.string();
            auto labelPath = boost::replace_all_copy(inputPathStr, item,
                                                     "label");
            std::ifstream labelFile(labelPath);
            if (!labelFile.good())
            {
                std::cerr << "Failure reading " << labelPath << "\n";
                continue;
            }
            std::string label;
            std::getline(labelFile, label);
            labelFile.close();

            std::string sensorName = createSensorName(label, item, cpuId);

            auto findSensor = gCpuSensors.find(sensorName);
            if (findSensor != gCpuSensors.end())
            {
                if (debug)
                {
                    std::cout << "Will be replaced: " << inputPath << ": "
                              << sensorName << " is already created\n";
                }
                continue;
            }

            // check hidden properties
            bool show = true;
            for (const char* prop : hiddenProps)
            {
                if (label == prop)
                {
                    show = false;
                    break;
                }
            }

            /*
             * Find if there is DtsCritOffset is configured in config file
             * set it if configured or else set it to 0
             */
            double dtsOffset = 0;
            if (label == "DTS")
            {
                auto findThrOffset =
                    baseConfiguration->second.find("DtsCritOffset");
                if (findThrOffset != baseConfiguration->second.end())
                {
                    dtsOffset = std::visit(VariantToDoubleVisitor(),
                                           findThrOffset->second);
                }
            }

            const auto& it = sensorPropertiesMap.find(type);
            if (it == sensorPropertiesMap.end())
            {
                std::cerr
                    << "Failure getting sensor properties for sensor type: "
                    << type << "\n";
                continue;
            }
            const SensorProperties& prop = it->second;

            std::vector<thresholds::Threshold> sensorThresholds;
            std::string labelHead = label.substr(0, label.find(' '));
            parseThresholdsFromConfig(*sensorData, sensorThresholds,
                                      &labelHead);
            if (sensorThresholds.empty())
            {
                if (!parseThresholdsFromAttr(sensorThresholds, inputPathStr,
                                             prop.scaleFactor, dtsOffset))
                {
                    std::cerr << "error populating thresholds for "
                              << sensorName << "\n";
                }
            }
            auto& sensorPtr = gCpuSensors[sensorName];
            // make sure destructor fires before creating a new one
            sensorPtr = nullptr;
            sensorPtr = std::make_shared<IntelCPUSensor>(
                inputPathStr, sensorType, objectServer, dbusConnection, io,
                sensorName, std::move(sensorThresholds), *interfacePath, cpuId,
                show, dtsOffset, prop);
            sensorMapUpdated = true;
            createdSensors.insert(sensorName);
            if (debug)
            {
                std::cout << "Mapped: " << inputPath << " to " << sensorName
                          << "\n";
            }
        }
    }

    if (static_cast<unsigned int>(!createdSensors.empty()) != 0U)
    {
        std::cout << "Sensor" << (createdSensors.size() == 1 ? " is" : "s are")
                  << " created\n";
    }

    return true;
}

bool doWait(boost::asio::yield_context yield, const size_t delay)
{
    boost::system::error_code ec;
    waitTimer->expires_after(std::chrono::milliseconds(delay));
    waitTimer->async_wait(yield[ec]);
    if (ec == boost::asio::error::operation_aborted)
    {
        return false;
    }
    if (ec)
    {
        std::cerr << "Timer failed\n";
        return false;
    }
    return true;
}

void pollCPUSensors(boost::asio::yield_context yield)
{
    size_t failSensorCnt = 0;
    while (true)
    {
        sensorMapUpdated = false;

        size_t sensorCnt = gCpuSensors.size();
        if (sensorCnt == 0)
        {
            if (!doWait(yield, sensorEmptyWaitMs))
            {
                throw std::runtime_error("Wait timer failed");
            }
            continue;
        }

        size_t waitMs = sensorPollLoopMs / sensorCnt;
        for (auto& [name, sensor] : gCpuSensors)
        {
            sensor->setupRead(yield);
            if (!std::isfinite(sensor->value))
            {
                failSensorCnt++;
            }
            if (failSensorCnt >= failSensorLimit)
            {
                waitMs = sensorFailThrottleMs;
                failSensorCnt = 0;
            }
            else
            {
                waitMs = sensorPollLoopMs / sensorCnt;
            }
            if (!doWait(yield, waitMs))
            {
                throw std::runtime_error("Wait timer failed");
            }
            if (sensorMapUpdated)
            {
                break;
            }
        }
    }
}

bool exportDevice(const CPUConfig& config)
{
    std::ostringstream hex;
    hex << std::hex << config.addr;
    const std::string& addrHexStr = hex.str();
    std::string busStr = std::to_string(config.bus);

    std::string parameters = "peci-client 0x" + addrHexStr;
    std::string devPath = peciDevPath;
    std::string delDevice = devPath + "peci-" + busStr + "/delete_device";
    std::string newDevice = devPath + "peci-" + busStr + "/new_device";
    std::string newClient = devPath + busStr + "-" + addrHexStr + "/driver";

    std::filesystem::path devicePath(newDevice);
    const std::string& dir = devicePath.parent_path().string();
    for (const auto& path : std::filesystem::directory_iterator(dir))
    {
        if (!std::filesystem::is_directory(path))
        {
            continue;
        }

        const std::string& directoryName = path.path().filename();
        if (directoryName.starts_with(busStr) &&
            directoryName.ends_with(addrHexStr))
        {
            if (debug)
            {
                std::cout << parameters << " on bus " << busStr
                          << " is already exported\n";
            }

            std::ofstream delDeviceFile(delDevice);
            if (!delDeviceFile.good())
            {
                std::cerr << "Error opening " << delDevice << "\n";
                return false;
            }
            delDeviceFile << parameters;
            delDeviceFile.close();

            break;
        }
    }

    std::ofstream deviceFile(newDevice);
    if (!deviceFile.good())
    {
        std::cerr << "Error opening " << newDevice << "\n";
        return false;
    }
    deviceFile << parameters;
    deviceFile.close();

    if (!std::filesystem::exists(newClient))
    {
        std::cerr << "Error creating " << newClient << "\n";
        return false;
    }

    std::cout << parameters << " on bus " << busStr << " is exported\n";

    return true;
}

void detectCpu(boost::asio::steady_timer& pingTimer,
               boost::asio::steady_timer& creationTimer,
               boost::asio::io_context& io,
               sdbusplus::asio::object_server& objectServer,
               std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
               boost::container::flat_set<CPUConfig>& cpuConfigs,
               ManagedObjectType& sensorConfigs)
{
    size_t rescanDelaySeconds = 0;
    size_t pingSeconds = fastPingSeconds;
    static bool keepPinging = false;
    int peciFd = -1;

    for (CPUConfig& config : cpuConfigs)
    {
        if (config.state == State::READY)
        {
            // at least one CPU is found, use slowPing since other cpu may not
            // be present
            pingSeconds = slowPingSeconds;
            continue;
        }

        std::fstream rescan{rescanPath, std::ios::out};
        if (rescan.is_open())
        {
            std::vector<fs::path> peciPaths;
            std::ostringstream searchPath;
            searchPath << std::hex << "peci-" << config.bus << "/" << config.bus
                       << "-" << config.addr;
            findFiles(fs::path(peciDevPath + searchPath.str()),
                      R"(peci_cpu.dimmtemp.+/hwmon/hwmon\d+/name$)", peciPaths,
                      3);
            if (!peciPaths.empty())
            {
                config.state = State::READY;
                rescanDelaySeconds = 1;
            }
            else
            {
                findFiles(fs::path(peciDevPath + searchPath.str()),
                          R"(peci_cpu.cputemp.+/hwmon/hwmon\d+/name$)",
                          peciPaths, 3);
                if (!peciPaths.empty())
                {
                    config.state = State::ON;
                    rescanDelaySeconds = 3;
                }
                else
                {
                    // https://www.kernel.org/doc/html/latest/admin-guide/abi-testing.html#abi-sys-bus-peci-rescan
                    rescan << "1";
                }
            }
            if (config.state != State::READY)
            {
                keepPinging = true;
            }

            continue;
        }

        std::string peciDevPath = peciDev + config.busName;

        peci_SetDevName(peciDevPath.data());

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        if ((peci_Lock(&peciFd, PECI_NO_WAIT) != PECI_CC_SUCCESS) ||
            (peciFd < 0))
        {
            std::cerr << "unable to open " << peciDevPath << " "
                      << std::strerror(errno) << "\n";
            detectCpuAsync(pingTimer, slowPingSeconds, creationTimer, io,
                           objectServer, dbusConnection, cpuConfigs,
                           sensorConfigs);
            return;
        }

        State newState = State::OFF;

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        if (peci_Ping(config.addr) == PECI_CC_SUCCESS)
        {
            bool dimmReady = false;
            for (unsigned int rank = 0; rank < rankNumMax; rank++)
            {
                std::array<uint8_t, 8> pkgConfig{};
                uint8_t cc = 0;

                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
                if (peci_RdPkgConfig(config.addr, PECI_MBX_INDEX_DDR_DIMM_TEMP,
                                     rank, 4, &pkgConfig[0],
                                     &cc) == PECI_CC_SUCCESS)
                {
                    // Depending on CPU generation, both 0 and 0xFF can be used
                    // to indicate no DIMM presence
                    if (((pkgConfig[0] != 0xFF) && (pkgConfig[0] != 0U)) ||
                        ((pkgConfig[1] != 0xFF) && (pkgConfig[1] != 0U)))
                    {
                        dimmReady = true;
                        break;
                    }
                }
                else
                {
                    /* ping passed, read dimm temp failed*/
                    pingSeconds = slowPingSeconds;
                    break;
                }
            }

            if (dimmReady)
            {
                newState = State::READY;
            }
            else
            {
                newState = State::ON;
            }
        }

        if (config.state != newState)
        {
            if (newState != State::OFF)
            {
                if (config.state == State::OFF)
                {
                    std::array<uint8_t, 8> pkgConfig{};
                    uint8_t cc = 0;

                    if (peci_RdPkgConfig(config.addr, PECI_MBX_INDEX_CPU_ID, 0,
                                         4, &pkgConfig[0],
                                         &cc) == PECI_CC_SUCCESS)
                    {
                        std::cout << config.name << " is detected\n";
                        if (!exportDevice(config))
                        {
                            pingSeconds = failPingSeconds;
                            newState = State::OFF;
                        }
                    }
                    else
                    {
                        /*ping passed, get CPUID failed*/
                        pingSeconds = slowPingSeconds;
                        newState = State::OFF;
                    }
                }

                if (newState == State::ON)
                {
                    rescanDelaySeconds = 3;
                }
                else if (newState == State::READY)
                {
                    rescanDelaySeconds = 5;
                    std::cout << "DIMM(s) on " << config.name
                              << " is/are detected\n";
                }
            }

            config.state = newState;
        }

        if (config.state != State::READY)
        {
            keepPinging = true;
        }

        if (debug)
        {
            std::cout << config.name << ", state: " << config.state << "\n";
        }
        peci_Unlock(peciFd);
    }

    if (rescanDelaySeconds != 0U)
    {
        creationTimer.expires_after(std::chrono::seconds(rescanDelaySeconds));
        creationTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }

            if (!createSensors(io, objectServer, dbusConnection, cpuConfigs,
                               sensorConfigs) ||
                keepPinging)
            {
                detectCpuAsync(pingTimer, slowPingSeconds, creationTimer, io,
                               objectServer, dbusConnection, cpuConfigs,
                               sensorConfigs);
            }
        });
    }
    else if (keepPinging)
    {
        detectCpuAsync(pingTimer, pingSeconds, creationTimer, io, objectServer,
                       dbusConnection, cpuConfigs, sensorConfigs);
    }
}

void detectCpuAsync(
    boost::asio::steady_timer& pingTimer, const size_t pingSeconds,
    boost::asio::steady_timer& creationTimer, boost::asio::io_context& io,
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& dbusConnection,
    boost::container::flat_set<CPUConfig>& cpuConfigs,
    ManagedObjectType& sensorConfigs)
{
    pingTimer.expires_after(std::chrono::seconds(pingSeconds));
    pingTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }

        detectCpu(pingTimer, creationTimer, io, objectServer, dbusConnection,
                  cpuConfigs, sensorConfigs);
    });
    if (debug)
    {
        std::cerr << "detectCpu in " << pingSeconds << " seconds.\n";
    }
}
std::optional<uint64_t> getPeciDeviceNum(const fs::path& peciAdapterNamePath)
{
    fs::path::iterator it = peciAdapterNamePath.begin();
    std::advance(it, 5); // /sys/bus/peci/devices/peci-xxxx
    std::string peciDeviceName = *it;
    auto pos = peciDeviceName.find('-');
    if (pos == std::string::npos)
    {
        std::cerr << "Incorrect PECI device name: " << peciDeviceName << "\n";
        return std::nullopt;
    }

    try
    {
        return std::stoull(peciDeviceName.substr(pos + 1, 1));
    }
    catch (std::logic_error&)
    {
        return std::nullopt;
    }
}

std::optional<std::string>
    readPeciAdapterNameFromFile(const fs::path& peciAdapterNamePath)
{
    std::ifstream nameFile(peciAdapterNamePath);
    if (!nameFile.good())
    {
        std::cerr << "Cannot read: " << peciAdapterNamePath << "\n";
        return std::nullopt;
    }

    std::string peciAdapterName;
    std::getline(nameFile, peciAdapterName);
    nameFile.close();
    if (peciAdapterName.empty())
    {
        return std::nullopt;
    }

    auto pos = peciAdapterName.find('-');
    peciAdapterName = peciAdapterName.substr(pos + 1);

    return peciAdapterName;
}

void addConfigsForOtherPeciAdapters(
    boost::container::flat_set<CPUConfig>& cpuConfigs, uint64_t& bus,
    uint64_t& addr, std::string& name, const State& state)
{
    std::vector<fs::path> peciAdapterNamePaths;
    if (!findFiles(fs::path(peciDevPath), R"(peci-\d+/name$)",
                   peciAdapterNamePaths, 1))
    {
        std::cerr << "No PECI adapters in system\n";
        return;
    }

    for (const fs::path& peciAdapterNamePath : peciAdapterNamePaths)
    {
        std::optional<uint64_t> peciDeviceNum =
            getPeciDeviceNum(peciAdapterNamePath);
        if (!peciDeviceNum || peciDeviceNum == bus)
        {
            continue;
        }

        std::optional<std::string> peciAdapterName =
            readPeciAdapterNameFromFile(peciAdapterNamePath);
        if (!peciAdapterName)
        {
            continue;
        }

        // Change result for peci-aspeed
        if (peciAdapterName->compare("bus") == 0)
        {
            peciAdapterName = "wire";
        }
        if (peciAdapterName->compare("i3c") == 0)
        {
            std::string p = peciAdapterNamePath.string();
            auto pos = p.find("peci-");
            std::string pecidev = p.substr(pos, 6);
            int cpu = peci_i3c_chardev_to_cpu(pecidev.c_str());

            if (int(addr) != cpu + 0x30)
            {
                continue;
            }
        }

        // WA:detectCpu uses cpuConfigs to send ping command
        // peci-mctp does not support ping command,
        // detectCPU will be refactored to support peci-mctp
        if (peciAdapterName->compare("mctp") != 0)
            cpuConfigs.emplace(*peciDeviceNum, addr,
                               name + "_" + *peciAdapterName, *peciAdapterName,
                               state);
    }
}

bool getCpuConfig(std::shared_ptr<sdbusplus::asio::connection>& systemBus,
                  boost::container::flat_set<CPUConfig>& cpuConfigs,
                  ManagedObjectType& sensorConfigs, boost::asio::io_context& io,
                  sdbusplus::asio::object_server& objectServer)
{
    bool useCache = false;
    sensorConfigs.clear();
    // use new data the first time, then refresh
    for (const char* type : sensorTypes)
    {
        if (!getSensorConfiguration(type, systemBus, sensorConfigs, useCache))
        {
            return false;
        }
        useCache = true;
    }

    // check PECI client addresses and names from CPU configuration
    // before starting ping operation
    for (const char* type : sensorTypes)
    {
        for (const auto& [path, cfgData] : sensorConfigs)
        {
            for (const auto& [intf, cfg] : cfgData)
            {
                if (intf != configInterfaceName(type))
                {
                    continue;
                }

                auto findName = cfg.find("Name");
                if (findName == cfg.end())
                {
                    continue;
                }
                std::string nameRaw = std::visit(VariantToStringVisitor(),
                                                 findName->second);
                std::string name = std::regex_replace(nameRaw, illegalDbusRegex,
                                                      "_");

                auto present = std::optional<bool>();
                // if we can't detect it via gpio, we set presence later
                for (const auto& [suppIntf, suppCfg] : cfgData)
                {
                    if (suppIntf.find("PresenceGpio") != std::string::npos)
                    {
                        present = cpuIsPresent(suppCfg);
                        break;
                    }
                }

                if (inventoryIfaces.find(name) == inventoryIfaces.end() &&
                    present)
                {
                    auto iface = objectServer.add_interface(
                        cpuInventoryPath + std::string("/") + name,
                        "xyz.openbmc_project.Inventory.Item");
                    iface->register_property("PrettyName", name);
                    iface->register_property("Present", *present);
                    iface->initialize();
                    inventoryIfaces[name] = std::move(iface);
                    if (*present)
                    {
                        // create required CPU sensors here in unavailable state
                        auto findRequiredTempSensor =
                            cfg.find("RequiredTempSensor");
                        auto findCpuId = cfg.find("CpuID");
                        if (findRequiredTempSensor != cfg.end() &&
                            findCpuId != cfg.end())
                        {
                            std::string label =
                                std::visit(VariantToStringVisitor(),
                                           findRequiredTempSensor->second);
                            // for temp sensor hwmon sysfs use input
                            std::string item{"input"};
                            int cpuId =
                                std::visit(VariantToUnsignedIntVisitor(),
                                           findCpuId->second);
                            std::string requiredSensorName =
                                createSensorName(label, item, cpuId);

                            auto& sensorPtr = gCpuSensors[requiredSensorName];
                            if (sensorPtr == nullptr)
                            {
                                // created a dummy sensor for required sensor,
                                // will be replaced with a real one if it is
                                // detected
                                std::string objectType{};
                                std::vector<thresholds::Threshold>
                                    emptyThreshold{};
                                std::string emptyConfig{};
                                sensorPtr = std::make_unique<IntelCPUSensor>(
                                    objectType, objectServer, systemBus, io,
                                    requiredSensorName,
                                    std::move(emptyThreshold), emptyConfig);
                                std::cout << "created required CPU sensor "
                                          << requiredSensorName << "\n";
                            }
                        }
                    }
                }
                auto findBus = cfg.find("Bus");
                if (findBus == cfg.end())
                {
                    std::cerr << "Can't find 'Bus' setting in " << name << "\n";
                    continue;
                }
                uint64_t bus = std::visit(VariantToUnsignedIntVisitor(),
                                          findBus->second);

                auto findAddress = cfg.find("Address");
                if (findAddress == cfg.end())
                {
                    std::cerr << "Can't find 'Address' setting in " << name
                              << "\n";
                    continue;
                }
                uint64_t addr = std::visit(VariantToUnsignedIntVisitor(),
                                           findAddress->second);

                if (debug)
                {
                    std::cout << "bus: " << bus << "\n";
                    std::cout << "addr: " << addr << "\n";
                    std::cout << "name: " << name << "\n";
                    std::cout << "type: " << type << "\n";
                }

                cpuConfigs.emplace(bus, addr, name, "default", State::OFF);
                addConfigsForOtherPeciAdapters(cpuConfigs, bus, addr, name,
                                               State::OFF);
            }
        }
    }

    if (static_cast<unsigned int>(!cpuConfigs.empty()) != 0U)
    {
        std::cout << "CPU config" << (cpuConfigs.size() == 1 ? " is" : "s are")
                  << " parsed\n";
        return true;
    }

    return false;
}

void udevAction(udev_device* dev,
                boost::container::flat_set<CPUConfig>& cpuConfigs)
{
    std::string action = udev_device_get_action(dev);
    if (action.empty())
    {
        return;
    }

    std::string devpath = udev_device_get_devpath(dev);

    auto pos = devpath.find("peci-");
    std::string pecidevName = devpath.substr(pos);

    pos = pecidevName.find("/");
    std::string peciAdapterName = pecidevName.substr(5, pos - 5);
    pecidevName = pecidevName.substr(pos + 1);

    pos = pecidevName.find("/");
    if (pos != std::string::npos)
    {
        return; // skip peci-client paths
    }

    pos = pecidevName.find("-");
    int bus = std::stoi(pecidevName.substr(pos + 1));

    pos = peciAdapterName.find(".");
    if (pos != std::string::npos)
    {
        peciAdapterName = peciAdapterName.substr(0, pos);
    }

    if (action.compare("add") == 0)
    {
        for (CPUConfig& config : cpuConfigs)
        {
            if (config.bus != 0)
            {
                continue;
            }

            if (peciAdapterName.compare("i3c") == 0)
            {
                int cpu = peci_i3c_chardev_to_cpu(pecidevName.c_str());

                if (int(config.addr) != cpu + 0x30)
                {
                    continue;
                }
            }
            cpuConfigs.emplace(bus, config.addr,
                               config.name + "_" + peciAdapterName,
                               peciAdapterName, State::OFF);
        }
    }
    else if (action.compare("remove") == 0)
    {
        for (CPUConfig& config : cpuConfigs)
        {
            if (config.bus == bus)
            {
                cpuConfigs.erase(config);
            }
        }
    }
}
void udevAsync(boost::asio::posix::stream_descriptor& udevDescriptor,
               udev_monitor* umonitor,
               boost::container::flat_set<CPUConfig>& cpuConfigs)
{
    udevDescriptor.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [&udevDescriptor, umonitor,
         &cpuConfigs](const boost::system::error_code& ec) {
            if (ec)
            {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return;
                }

                std::cerr << "error reading uevents: " << ec.message().c_str()
                          << "\n";

                udevAsync(udevDescriptor, umonitor, cpuConfigs);
                return;
            }

            udev_device* dev = udev_monitor_receive_device(umonitor);
            if (dev)
            {
                udevAction(dev, cpuConfigs);
                udev_device_unref(dev);
            }
            else
            {
                std::cerr << "udevmonitor get device failed: "
                          << ec.message().c_str() << "\n";
            }

            udevAsync(udevDescriptor, umonitor, cpuConfigs);
        });
}

int main()
{
    boost::asio::io_context io;
    auto systemBus = std::make_shared<sdbusplus::asio::connection>(io);
    boost::container::flat_set<CPUConfig> cpuConfigs;

    sdbusplus::asio::object_server objectServer(systemBus, true);
    objectServer.add_manager("/xyz/openbmc_project/sensors");
    boost::asio::steady_timer pingTimer(io);
    boost::asio::steady_timer creationTimer(io);
    boost::asio::steady_timer filterTimer(io);
    waitTimer = std::make_unique<boost::asio::steady_timer>(io);
    ManagedObjectType sensorConfigs;

    udev* udevContext;
    udev_monitor* umonitor;
    boost::asio::posix::stream_descriptor udevDescriptor(io);

    filterTimer.expires_after(std::chrono::seconds(1));
    filterTimer.async_wait([&](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }

        if (getCpuConfig(systemBus, cpuConfigs, sensorConfigs, io,
                         objectServer))
        {
            detectCpuAsync(pingTimer, fastPingSeconds, creationTimer, io,
                           objectServer, systemBus, cpuConfigs, sensorConfigs);
        }
    });

    std::function<void(sdbusplus::message_t&)> eventHandler =
        [&](sdbusplus::message_t& message) {
        if (message.is_method_error())
        {
            std::cerr << "callback method error\n";
            return;
        }

        if (debug)
        {
            std::cout << message.get_path() << " is changed\n";
        }

        // this implicitly cancels the timer
        filterTimer.expires_after(std::chrono::seconds(1));
        filterTimer.async_wait([&](const boost::system::error_code& ec) {
            if (ec == boost::asio::error::operation_aborted)
            {
                return; // we're being canceled
            }

            if (getCpuConfig(systemBus, cpuConfigs, sensorConfigs, io,
                             objectServer))
            {
                detectCpuAsync(pingTimer, fastPingSeconds, creationTimer, io,
                               objectServer, systemBus, cpuConfigs,
                               sensorConfigs);
            }
        });
    };

    udevContext = udev_new();
    if (!udevContext)
    {
        std::cerr << "can't create udev library context\n";
    }
    umonitor = udev_monitor_new_from_netlink(udevContext, "udev");
    if (!umonitor)
    {
        std::cerr << "can't create udev monitor\n";
    }
    udev_monitor_filter_add_match_subsystem_devtype(umonitor, "peci", NULL);
    udev_monitor_enable_receiving(umonitor);

    udevDescriptor.assign(udev_monitor_get_fd(umonitor));
    udevAsync(udevDescriptor, umonitor, cpuConfigs);

    std::vector<std::unique_ptr<sdbusplus::bus::match_t>> matches =
        setupPropertiesChangedMatches(*systemBus, sensorTypes, eventHandler);

    systemBus->request_name("xyz.openbmc_project.IntelCPUSensor");

    setupManufacturingModeMatch(*systemBus);
    boost::asio::spawn(
        io, [](boost::asio::yield_context yield) { pollCPUSensors(yield); });
    io.run();
    return 0;
}
