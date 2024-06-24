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
    const std::string& sensorConfiguration) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objServer(objectServer), waitTimer(io), conn(conn)
{
    sensorInterface = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/bmcfirmwarehealth/" + name,
        "xyz.openbmc_project.Sensor.State");

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/bmcfirmwarehealth/" + name,
        association::interface);
    setInitialProperties();
}

BMCFirmwareHealth::~BMCFirmwareHealth()
{
    objServer.remove_interface(sensorInterface);
    objServer.remove_interface(association);
}

void BMCFirmwareHealth::setupRead(void)
{
    monitorState();
}

void BMCFirmwareHealth::monitorState()
{
    uint16_t state = 0;

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
                double value = std::visit(VariantToDoubleVisitor(),
                                          findValue->second);
                if (std::isnan(value))
                {
                    state = state |
                            (1 << managementSubsystemHealth::sensorUnavailable);
                }
                else if (value == 0)
                {
                    state = state |
                            (1 << managementSubsystemHealth::sensorFailure);
                }
            }
        }
    }

    updateState(sensorInterface, state);
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
        std::shared_ptr<BMCFirmwareHealth> self = weakRef.lock();
        if (!self)
        {
            return;
        }
        self->setupRead();
    });
}
