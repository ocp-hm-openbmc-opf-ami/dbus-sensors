#include <unistd.h>

#include <ProcessorStatus.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

ProcessorStatus::ProcessorStatus(
    sdbusplus::asio::object_server& objectServer,
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    boost::asio::io_context& io, const std::string& sensorName,
    const std::string& gpioName, const std::string& sensorConfiguration) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    gpio(gpioName), objServer(objectServer),procPresentEvent(io)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/cpu/" + name,
        "xyz.openbmc_project.Sensor.State");

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/cpu/" + name,
        association::interface);
    setInitialProperties();

    setupEvent(conn, gpioName, procPresentLine, procPresentEvent);
}

ProcessorStatus::~ProcessorStatus()
{
    objServer.remove_interface(sensorInterface);
}

bool ProcessorStatus::setupEvent(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::string& procGpioName, gpiod::line& gpioLine,
    boost::asio::posix::stream_descriptor& gpioEventDescriptor)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(procGpioName);
    if (!gpioLine)
    {
        std::cerr << "Failed to find the line\n";

        return false;
    }

    try
    {
        gpioLine.request({"proc-sensor", gpiod::line_request::EVENT_BOTH_EDGES,
                          gpiod::line_request::FLAG_ACTIVE_LOW});
    }
    catch (std::exception&)
    {
        std::cerr << "Failed to request events\n";
        return false;
    }

    bool state = (gpioLine.get_value() == 1);
    std::vector<std::string> logData;

    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        std::cerr << "Failed to get fd\n";
        return false;
    }

    gpioEventDescriptor.assign(gpioLineFd);

    logData.push_back(name);
    logData.push_back("Presence Detected");
    logData.push_back(processorPath + name);
    logData.push_back("SensorProcessorPresence");
    if (state)
    {
        addSelEntry(conn, logData, procPresence, state);
        updateState(sensorInterface,(static_cast<uint16_t>(1 << static_cast<uint16_t>(CpuEvent::PresenceDetected))));
    }

    monitor(conn, logData, procPresence, gpioEventDescriptor, gpioLine);

    return true;
}

void ProcessorStatus::monitor(
    std::shared_ptr<sdbusplus::asio::connection>& conn,
    const std::vector<std::string>& logData,
    const std::vector<uint8_t> procPresence,
    boost::asio::posix::stream_descriptor& event, gpiod::line& line)
{

    event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [this, &conn, &event, &line, &logData,
         &procPresence](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << " fd handler error: " << ec.message() << "\n";
                return;
            }
            gpiod::line_event lineEvent = line.event_read();
            if( lineEvent.event_type == gpiod::line_event::FALLING_EDGE)
            {
            updateState(sensorInterface,(static_cast<uint16_t>(1 << static_cast<uint16_t>(CpuEvent::PresenceDetected))));
            addSelEntry(conn, logData, procPresence,
                        lineEvent.event_type ==
                            gpiod::line_event::FALLING_EDGE);
            }
            // Start monitoring for next event
            monitor(conn, logData, procPresence, event, line);
        });
}
