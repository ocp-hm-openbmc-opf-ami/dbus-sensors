// @author Gayathri D gayathrid@ami.com

#pragma once

#include "SensorInfo.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <map>
#include <regex>
#include <string>

inline int findsensoridx(std::string_view path)
{
    // Exact match first
    for (std::size_t i = 0; i < Sensor_Info.size(); ++i)
    {
        if (path == Sensor_Info[i].SensorName)
        {
            return static_cast<int>(i);
        }
    }

    // Partial match fallback - find longest matching substring
    // Require match at a word boundary to avoid CPU1_DIMM_A matching
    // CPU1_DIMM_AF
    int bestIdx = -1;
    size_t bestLen = 0;
    for (std::size_t i = 0; i < Sensor_Info.size(); ++i)
    {
        const auto& sname = Sensor_Info[i].SensorName;
        if (sname.empty())
        {
            continue;
        }
        auto pos = path.find(sname);
        if (pos == std::string_view::npos)
        {
            continue;
        }
        // Ensure match ends at end of path or is followed by a
        // non-alphanumeric character (word boundary)
        size_t endPos = pos + sname.length();
        if (endPos < path.length() && std::isalnum(path[endPos]))
        {
            continue;
        }
        if (sname.length() > bestLen)
        {
            bestIdx = static_cast<int>(i);
            bestLen = sname.length();
        }
    }

    return bestIdx;
}

inline void ReadSensorInfo(std::shared_ptr<sdbusplus::asio::connection> bus)
{
    static constexpr const char* kService = "xyz.openbmc_project.EntityManager";
    static constexpr const char* kIface =
        "xyz.openbmc_project.Configuration.NmSensor";

    std::string BaseBoard = findBoardName(bus);
    if (BaseBoard.empty())
    {
        std::fprintf(stderr, "ReadSensorInfo: Board name not found\n");
        return;
    }

    std::string kBasePath =
        "/xyz/openbmc_project/inventory/system/board/" + BaseBoard;

    // Discover NmSensor objects dynamically from EntityManager
    using SubTree =
        std::map<std::string, std::map<std::string, std::vector<std::string>>>;

    SubTree subtree;
    try
    {
        auto m = bus->new_method_call(
            "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper", "GetSubTree");
        m.append(kBasePath, int32_t{1}, std::vector<std::string>{kIface});

        auto reply = bus->call(m);
        reply.read(subtree);
    }
    catch (const sdbusplus::exception::SdBusError& e)
    {
        std::fprintf(
            stderr,
            "ReadSensorInfo: Failed to enumerate NmSensor objects: %s\n",
            e.what());
        return;
    }

    if (subtree.empty())
    {
        std::fprintf(stderr, "ReadSensorInfo: No NmSensor objects found\n");
        return;
    }

    // Build Sensor_Info from discovered objects
    Sensor_Info.clear();
    Sensor_Info.resize(subtree.size());

    std::size_t idx = 0;
    for (const auto& [objPath, serviceMap] : subtree)
    {
        std::string sensorName =
            std::filesystem::path(objPath).filename().string();

        Sensor_Info[idx].SensorName = sensorName;

        // Use synchronous D-Bus calls to ensure Sensor_Info is fully
        // populated before any sensor property getter can read it.
        try
        {
            auto m2 =
                bus->new_method_call(kService, objPath.c_str(),
                                     "org.freedesktop.DBus.Properties", "Get");
            m2.append(std::string(kIface), std::string("SensorNumber"));
            auto reply2 = bus->call(m2);
            std::variant<uint64_t> val;
            reply2.read(val);
            Sensor_Info[idx].SensorNumber = std::get<uint64_t>(val);
        }
        catch (const sdbusplus::exception::SdBusError& e)
        {
            std::fprintf(stderr, "Couldn't get sensor number %s: %s\n",
                         sensorName.c_str(), e.what());
        }

        try
        {
            auto m3 =
                bus->new_method_call(kService, objPath.c_str(),
                                     "org.freedesktop.DBus.Properties", "Get");
            m3.append(std::string(kIface), std::string("LUN"));
            auto reply3 = bus->call(m3);
            std::variant<uint64_t> lunVal;
            reply3.read(lunVal);
            Sensor_Info[idx].Lun = std::get<uint64_t>(lunVal);
        }
        catch (const sdbusplus::exception::SdBusError& e)
        {
            std::fprintf(stderr, "Couldn't get lun number %s: %s\n",
                         sensorName.c_str(), e.what());
        }

        ++idx;
    }
}

inline std::string findBoardName(
    std::shared_ptr<sdbusplus::asio::connection> bus)
{
    if (!bus)
    {
        std::fprintf(stderr, "findBoardName: bus is null\n");
        return "";
    }

    using SubTree =
        std::map<std::string, std::map<std::string, std::vector<std::string>>>;

    try
    {
        auto m = bus->new_method_call(
            "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper", "GetSubTree");

        const std::string root = "/xyz/openbmc_project/inventory/system/board";
        const int32_t depth = 1;
        const std::vector<std::string> ifaces = {
            "xyz.openbmc_project.Inventory.Item.Board"};
        m.append(root, depth, ifaces);

        SubTree subtree;
        auto reply = bus->call(m);
        reply.read(subtree);

        if (subtree.empty())
            return "";

        for (const auto& [objPath, _] : subtree)
        {
            std::string name =
                std::filesystem::path(objPath).filename().string();
            if (name == "APISensor")
                continue;
            return name;
        }
        return "";
    }
    catch (const sdbusplus::exception::SdBusError& e)
    {
        std::fprintf(stderr, "findBoardName: D-Bus error: %s\n", e.what());
        return "";
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "findBoardName: exception: %s\n", e.what());
        return "";
    }
}
