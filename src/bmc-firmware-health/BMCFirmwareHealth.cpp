#include <unistd.h>

#include <BMCFirmwareHealth.hpp>

#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

BMCFirmwareHealth::BMCFirmwareHealth(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    const std::string& sensorConfiguration, uint16_t sensorNumber,
    uint8_t lun) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn, sensorNumber,
             lun),
    objServer(objectServer), waitTimer(io), conn(conn)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/bmcfirmwarehealth/" + name,
        "xyz.openbmc_project.Sensor.State");

    if (!sensorInterface)
    {
        std::cerr << "Error: Failed to create DBus interfaces\n";
        return;
    }

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/bmcfirmwarehealth/" + name,
        association::interface);

    if (!association)
    {
        std::cerr << "Error: Failed to create DBus interfaces\n";
        return;
    }

    setInitialProperties();

    if (!sensorInterface->initialize() || !association->initialize())
    {
        std::cerr << "Error: Failed to initialize DBus interfaces\n";
        return;
    }
    setupRead();
}

BMCFirmwareHealth::~BMCFirmwareHealth()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

bool BMCFirmwareHealth::isFRUAccessible()
{
    auto fruAvailableCall =
        conn->new_method_call(fruService, fruObjectPath, PROP_INTF, "GetAll");
    fruAvailableCall.append(fruIntf);

    try
    {
        auto fruAvailableReply = conn->call(fruAvailableCall);
        return true;
    }
    catch (const std::exception& e)
    {
        return false;
    }
}

void BMCFirmwareHealth::setupRead(void)
{
    monitorState();
}

void BMCFirmwareHealth::monitorState()
{
    uint8_t oldValue = state;

    // Create new state based on current conditions (don't start by clearing)
    uint8_t newState = state;

    // Local variables for SEL entries
    std::vector<uint8_t> eventData(selEvtDataMaxSize, 0xFF);
    std::vector<std::string> logData(logDataMaxSize);

    // setup connection to dbus
    boost::asio::io_context io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);

    auto mapper = conn->new_method_call(mapper::busName, mapper::path,
                                        mapper::interface, mapper::subtree);
    mapper.append("/", 0, std::array<const char*, 1>{senInterface});
    std::unordered_map<
        std::string, std::unordered_map<std::string, std::vector<std::string>>>
        respData;

    try
    {
        auto resp = conn->call(mapper);
        resp.read(respData);
    }
    catch (const sdbusplus::exception_t&)
    {
        std::cerr << "Populate Failures Mapper Error\n";
        return;
    }

    for (const auto& [path, interfaceDict] : respData)
    {
        if (path.find("PMT") != std::string::npos)
        {
            continue; // Skip PMT-related sensor paths
        }

        for (const auto& [owner, _] : interfaceDict)
        {
            auto call = conn->new_method_call(owner.c_str(), path.c_str(),
                                              PROP_INTF, METHOD_GET_ALL);
            call.append(senInterface);
            boost::container::flat_map<std::string,
                                       std::variant<double, int64_t>>
                values;
            try
            {
                auto data = conn->call(call);
                data.read(values);
            }
            catch (const sdbusplus::exception_t&)
            {
                std::cerr << "Populate Failures Mapper Error\n";
                return;
            }

            auto findValue = values.find("Value");
            if (findValue != values.end())
            {
                double value =
                    std::visit(VariantToDoubleVisitor(), findValue->second);
                if (std::isnan(value))
                {
                    newState |=
                        (1 << managementSubsystemHealth::sensorUnavailable);
                }
                else if (value == 0)
                {
                    newState |= (1 << managementSubsystemHealth::sensorFailure);
                }
            }
        }
    }

    // --- Transition-aware FRU unavailable bit ---
    if (isFRUAccessible())
    {
        // Clear the bit if FRU is available
        newState &=
            ~(1 << managementSubsystemHealth::controllerAccessUnavailable);
    }
    else
    {
        // Set the bit if FRU is unavailable
        newState |=
            (1 << managementSubsystemHealth::controllerAccessUnavailable);
    }

    // Check System Lock status
    try
    {
        auto mapperCall = conn->new_method_call(mapper::busName, mapper::path,
                                                mapper::interface, "GetObject");
        mapperCall.append(systemLockObjectPath,
                          std::vector<std::string>{systemLockIntf});

        std::map<std::string, std::vector<std::string>> owners;
        auto mapperReply = conn->call(mapperCall);
        mapperReply.read(owners);

        if (!owners.empty())
        {
            auto service = owners.begin()->first;

            auto call = conn->new_method_call(
                service.c_str(), systemLockObjectPath, PROP_INTF, "Get");
            call.append(systemLockIntf, systemLockProp);

            std::variant<bool> value;
            auto reply = conn->call(call);
            reply.read(value);

            bool locked = std::get<bool>(value);
            if (locked)
            {
                newState |=
                    (1
                     << managementSubsystemHealth::ManagementControllerOffline);
            }
            else
            {
                newState &= ~(
                    1
                    << managementSubsystemHealth::ManagementControllerOffline);
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "SystemLock polling failed: " << e.what() << "\n";
    }

    // Update state with the newly calculated state
    state = newState;

    // Log SEL and update DBus only when state actually changes
    if (oldValue != state)
    {
        updateState(sensorInterface, state);

        uint8_t asserted = static_cast<uint8_t>((~oldValue) & state);
        logData[0] = name;
        logData[2] = "/xyz/openbmc_project/sensors/bmcfirmwarehealth/" + name;
        logData[3] = "SensorHealthStateAssert";

        if (asserted & (1 << managementSubsystemHealth::sensorFailure))
        {
            eventData[0] = static_cast<uint8_t>(sensorFailure);
            logData[1] = "sensorFailure";
            addSelEntry(conn, logData, eventData, true, sensorNumber);
        }
        else if (asserted & (1 << managementSubsystemHealth::sensorUnavailable))
        {
            eventData[0] = static_cast<uint8_t>(sensorUnavailable);
            logData[1] = "sensorUnavailable";
            addSelEntry(conn, logData, eventData, true, sensorNumber);
        }
        else if (asserted &
                 (1 << managementSubsystemHealth::controllerAccessUnavailable))
        {
            eventData[0] = static_cast<uint8_t>(controllerAccessUnavailable);
            logData[1] = "controllerAccessUnavailable";
            addSelEntry(conn, logData, eventData, true, sensorNumber);
        }
        else if (asserted &
                 (1 << managementSubsystemHealth::ManagementControllerOffline))
        {
            eventData[0] = static_cast<uint8_t>(ManagementControllerOffline);
            logData[1] = "ManagementControllerOffline";
            addSelEntry(conn, logData, eventData, true, sensorNumber);
        }
    }

    // Save current state for next cycle
    prevState = state;
    restartRead();
}

void BMCFirmwareHealth::restartRead()
{
    std::weak_ptr<BMCFirmwareHealth> weakRef = weak_from_this();
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([weakRef](const boost::system::error_code& ec) {
        if (ec == boost::asio::error::operation_aborted)
        {
            return; // we're being canceled
        }
        if (ec)
        {
            std::cerr << "error in restartRead\n" << std::endl;
            return;
        }
        std::shared_ptr<BMCFirmwareHealth> self = weakRef.lock();
        if (!self)
        {
            return;
        }
        self->setupRead();
    });
}
