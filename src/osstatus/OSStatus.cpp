#include <unistd.h>

#include <OSStatus.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

OSStatus::OSStatus(sdbusplus::asio::object_server& objectServer,
                   std::shared_ptr<sdbusplus::asio::connection>& conn,
                   const std::string& sensorName,
                   const std::string& sensorConfiguration) :
    Discrete(escapeName(sensorName), sensorConfiguration, conn),
    objServer(objectServer)
{
    sensorInterface =
        objectServer.add_interface("/xyz/openbmc_project/sensors/os/" + name,
                                   "xyz.openbmc_project.Sensor.State");

    association = objectServer.add_interface(
        "/xyz/openbmc_project/sensors/os/" + name, association::interface);
    setInitialProperties();
}

OSStatus::~OSStatus()
{
    objServer.remove_interface(sensorInterface);
}
