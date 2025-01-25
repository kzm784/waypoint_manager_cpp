#ifndef LOAD_WAYPOINTS_HPP_
#define LOAD_WAYPOINTS_HPP_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

class LoadWaypoints
{
public:
    LoadWaypoints() {}

    std::vector<std::vector<std::string>> loadWaypointsFromCSV(const std::string& csv_path)
    {
        std::ifstream file(csv_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("LoadWaypoints"), "Failed to open file: %s", csv_path.c_str());
            return {};
        }

        std::vector<std::vector<std::string>> waypoints_data;
        std::string line;
        bool is_header = true;

        while (std::getline(file, line))
        {
            std::stringstream line_stream(line);
            std::vector<std::string> row;
            std::string cell;

            if (is_header)
            {
                is_header = false;
                continue;
            }

            while (std::getline(line_stream, cell, ','))
            {
                row.push_back(cell);
            }

            waypoints_data.push_back(row);
        }

		RCLCPP_INFO(rclcpp::get_logger("LoadWaypoints"), "Loaded %ld waypoints from CSV", waypoints_data.size());
        return waypoints_data;
    }
};

#endif // LOAD_WAYPOINTS_HPP_
