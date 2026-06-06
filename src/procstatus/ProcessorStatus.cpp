#include <unistd.h>

#include <ProcessorStatus.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

// ProcessorStatus
static const std::map<size_t, CpuEvent> indexToEvent = {
    {0, CpuEvent::IERR},
    {1, CpuEvent::ThermalTrip},
    {2, CpuEvent::FRB1},
    {3, CpuEvent::FRB2},
    {7, CpuEvent::PresenceDetected},
    {10, CpuEvent::Throttled},
    {11, CpuEvent::UncorrectableMachineCheckException},
};

const std::map<CpuEvent, std::string> eventDescriptionMap = {
    {CpuEvent::PresenceDetected, "Presence Detected"},
    {CpuEvent::IERR, "IERR"},
    {CpuEvent::ThermalTrip, "Thermal Trip"},
    {CpuEvent::FRB1, "FRB1/BIST failure"},
    {CpuEvent::FRB2, "FRB2/Hang in post failure"},
    {CpuEvent::Throttled, "Throttled"},
    {CpuEvent::UncorrectableMachineCheckException,
     "Uncorrectable machine check exception"}};

std::vector<uint8_t> getSelEventData(CpuEvent event)
{
    std::vector<uint8_t> data(ProcessorselEvtDataMaxSize, 0x00);
    data[0] = static_cast<uint8_t>(event); // Offset
    data[1] = 0xFF;
    data[2] = 0xFF;
    return data;
}

std::string getEventDescription(CpuEvent event)
{
    auto it = eventDescriptionMap.find(event);
    return (it != eventDescriptionMap.end()) ? it->second : "Unknown";
}

ProcessorStatus::ProcessorStatus(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    const std::vector<std::string>& gpioNames,
    const std::string& sensorConfiguration,
    const std::vector<std::string>& dbusPaths,
    const std::vector<std::string>& dbusIfaces,
    const std::vector<std::string>& dbusProperties, bool dbusEnabled,
    uint16_t sensorNumber, uint8_t lun) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn, sensorNumber,
             lun),
    objServer(objectServer), dbus(dbusEnabled), waitTimer(io), conn(conn),
    DBusPaths(dbusPaths), DBusIfaces(dbusIfaces), DBusProperties(dbusProperties)
{
    sensorInterface =
        objectServer.add_interface("/xyz/openbmc_project/sensors/cpu/" + name,
                                   "xyz.openbmc_project.Sensor.State");

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/cpu/" + name, association::interface);

    if (!sensorInterface || !association)
    {
        return;
    }

    setInitialProperties();

    hasDbusConfig = dbus;

    for (size_t index = 0; index < gpioNames.size(); ++index)
    {
        const std::string& gpioName = gpioNames[index];

        if (gpioName.empty())
        {
            continue;
        }

        auto it = indexToEvent.find(index);
        if (it == indexToEvent.end())
        {
            continue;
        }

        CpuEvent eventType = it->second;
        gpiod::line line;
        boost::asio::posix::stream_descriptor descriptor(io);
        if (setupEvent(conn, gpioName, line, descriptor, eventType))
        {
            gpioLines.push_back(std::move(line));
            gpioEventDescriptors.push_back(std::move(descriptor));
            gpioEventTypes.push_back(eventType);
            gpioEventMap[gpioName] = eventType;
        }
    }

    hasGpioConfig = !gpioLines.empty();

    if (hasDbusConfig)
    {
        monitorDbus();
    }
    if (hasGpioConfig)
    {
        pollGpioStates();
    }

    if (hasDbusConfig || hasGpioConfig)
    {
        restartRead();
    }
}

ProcessorStatus::~ProcessorStatus()
{
    objServer.remove_interface(sensorInterface);
}

bool ProcessorStatus::setupEvent(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& procGpioName, gpiod::line& gpioLine,
    boost::asio::posix::stream_descriptor& gpioEventDescriptor,
    CpuEvent eventType)
{
    gpioLine = gpiod::find_line(procGpioName);
    if (!gpioLine)
    {
        return false;
    }

    try
    {
        gpioLine.request({"proc-sensor", gpiod::line_request::EVENT_BOTH_EDGES,
                          gpiod::line_request::FLAG_ACTIVE_LOW});
    }
    catch (const std::exception& e)
    {
        std::cerr << "GPIO request failed: " << e.what() << "\n";
        return false;
    }

    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        return false;
    }

    gpioEventDescriptor.assign(gpioLineFd);

    // Read initial GPIO state and set bits accordingly
    bool state = (gpioLine.get_value() == 1);
    if (state)
    {
        uint16_t oldValue = currentState;
        currentState |= (1 << static_cast<uint16_t>(eventType));

        if (oldValue != currentState)
        {
            updateState(sensorInterface, currentState);

            std::vector<std::string> logData = {
                name, getEventDescription(eventType),
                "/xyz/openbmc_project/sensors/cpu/" + name, "ProcessorStatus"};

            auto eventData = getSelEventData(eventType);
            addSelEntry(conn, logData, eventData, true, sensorNumber);
        }
    }

    monitor(conn, gpioEventDescriptor, gpioLine, eventType);
    return true;
}

void ProcessorStatus::monitor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::posix::stream_descriptor& event, gpiod::line& line,
    CpuEvent eventType)
{
    event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [this, &conn, &event, &line,
         eventType](const boost::system::error_code& ec) {
            if (ec || !line.is_requested())
                return;

            gpiod::line_event lineEvent = line.event_read();
            uint16_t oldState = currentState;

            if (lineEvent.event_type == gpiod::line_event::FALLING_EDGE)
            {
                currentState |= (1 << static_cast<uint16_t>(eventType));
            }
            else if (lineEvent.event_type == gpiod::line_event::RISING_EDGE)
            {
                currentState &= ~(1 << static_cast<uint16_t>(eventType));
            }

            // Only log SEL and update state if it changed
            if (oldState != currentState)
            {
                updateState(sensorInterface, currentState);

                std::vector<std::string> logData = {
                    name, getEventDescription(eventType),
                    "/xyz/openbmc_project/sensors/cpu/" + name,
                    "ProcessorStatus"};

                auto eventData = getSelEventData(eventType);
                addSelEntry(conn, logData, eventData, true, sensorNumber);
            }

            monitor(conn, event, line, eventType);
        });
}

void ProcessorStatus::monitorDbus()
{
    uint16_t oldValue = currentState;
    uint16_t newState = currentState;

    for (size_t index = 0; index < DBusPaths.size(); ++index)
    {
        if (DBusPaths[index].empty() || DBusIfaces[index].empty() ||
            DBusProperties[index].empty() ||
            indexToEvent.find(index) == indexToEvent.end())
        {
            continue;
        }

        CpuEvent event = indexToEvent.at(index);
        // DBus bit reflects current DBus property state each cycle
        newState &= ~(1 << static_cast<uint16_t>(event));

        try
        {
            auto service =
                getService(DBusIfaces[index].c_str(), DBusPaths[index].c_str());
            auto method =
                conn->new_method_call(service.c_str(), DBusPaths[index].c_str(),
                                      "org.freedesktop.DBus.Properties", "Get");
            method.append(DBusIfaces[index]);
            method.append(DBusProperties[index]);

            auto reply = conn->call(method);
            if (reply.is_method_error())
            {
                log<level::ERR>("GetAll failed");
                continue;
            }

            std::variant<bool> result;
            reply.read(result);

            bool value = std::get<bool>(result);

            // Assert event only when DBus property is false
            if (!value)
            {
                newState |= (1 << static_cast<uint16_t>(event));
            }
        }
        catch (sdbusplus::exception_t& e)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Failed to fetch",
                phosphor::logging::entry("EXCEPTION=%s", e.what()));
        }
    }

    // Update state and log only if changed
    if (oldValue != newState)
    {
        currentState = newState;
        updateState(sensorInterface, currentState);

        // Log SEL for each new asserted bit
        uint16_t asserted = (~oldValue) & newState;
        for (const auto& [index, event] : indexToEvent)
        {
            if (asserted & (1 << static_cast<uint16_t>(event)))
            {
                std::vector<std::string> logData = {
                    name, getEventDescription(event),
                    "/xyz/openbmc_project/sensors/cpu/" + name,
                    "ProcessorStatus"};

                auto data = getSelEventData(event);
                addSelEntry(conn, logData, data, true, sensorNumber);
            }
        }
    }
}

void ProcessorStatus::restartRead()
{
    waitTimer.expires_after(std::chrono::milliseconds(sensorPollMs));
    waitTimer.async_wait([this](const boost::system::error_code& ec) {
        if (ec)
        {
            return;
        }

        if (hasDbusConfig)
        {
            this->monitorDbus();
        }
        if (hasGpioConfig)
        {
            this->pollGpioStates();
        }

        this->restartRead();
    });
}

void ProcessorStatus::pollGpioStates()
{
    uint16_t oldState = currentState;
    uint16_t newState = currentState;

    // Clear only GPIO-managed bits; keep DBus-managed bits intact
    for (CpuEvent event : gpioEventTypes)
    {
        newState &= ~(1 << static_cast<uint16_t>(event));
    }

    // Read current state of all GPIO lines
    for (size_t i = 0; i < gpioLines.size(); ++i)
    {
        if (!gpioLines[i].is_requested())
        {
            continue;
        }

        try
        {
            int value = gpioLines[i].get_value();
            if (value == 1) // Active (error present with ACTIVE_LOW)
            {
                CpuEvent event = gpioEventTypes[i];
                newState |= (1 << static_cast<uint16_t>(event));
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "GPIO poll error: " << e.what() << "\n";
        }
    }

    // Update state if changed
    if (oldState != newState)
    {
        currentState = newState;
        updateState(sensorInterface, currentState);

        // Log SEL for newly asserted bits
        uint16_t asserted = (~oldState) & newState;
        for (size_t i = 0; i < gpioEventTypes.size(); ++i)
        {
            CpuEvent event = gpioEventTypes[i];
            if (asserted & (1 << static_cast<uint16_t>(event)))
            {
                std::vector<std::string> logData = {
                    name, getEventDescription(event),
                    "/xyz/openbmc_project/sensors/cpu/" + name,
                    "ProcessorStatus"};

                auto eventData = getSelEventData(event);
                addSelEntry(conn, logData, eventData, true, sensorNumber);
            }
        }
    }
}
