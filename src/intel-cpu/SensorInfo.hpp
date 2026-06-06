// @author Gayathri D gayathrid@ami.com

#pragma once

#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <chrono>
#include <functional>
#include <regex>
#include <string>
#include <vector>

struct NM_Sensor_Info
{
    std::string SensorName;
    uint64_t SensorNumber = 0;
    uint64_t Lun = 0;
};

std::string findBoardName(std::shared_ptr<sdbusplus::asio::connection> bus);
void ReadSensorInfo(std::shared_ptr<sdbusplus::asio::connection> bus);
int findsensoridx(std::string_view path);

extern std::vector<NM_Sensor_Info> Sensor_Info;
