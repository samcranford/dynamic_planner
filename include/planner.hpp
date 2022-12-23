#pragma once

#include <math.h>
#include <iostream>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace planner {

struct Location {
    Location(double y_val = 0, double x_val = 0, bool json_global_val = false) : y(y_val), x(x_val), json_global(json_global_val) {
    }
    double y;
    double x;
    bool json_global;
};

constexpr int ROBOT_PIXEL_WIDTH = 10;                ///< width of the robot in map pixels for marking visited areas
constexpr double COSTMAP_RESOLUTION = .10;           ///< costmap resolution in meters/pixel
constexpr int FATAL_OCCUPANCY = 90;                  ///< occupancy value for fatal space in costmap, non-lethal such that travel is still permitted
constexpr int OUTSIDE_PROPERTY_OCCUPANCY = 75;       ///< occupancy value for outside property in costmap, percentage 0 - 100
constexpr int OUTSIDE_SECTION_OCCUPANCY = 25;        ///< occupancy value for outside section in costmap, percentage 0 - 100
constexpr int VISITED_OCCUPANCY = 10;                ///< occupancy value for visited space in costmap, percentage 0 - 100
constexpr int FREE_OCCUPANCY = 0;                    ///< occupancy value for free space in costmap, percentage 0 - 100
const int PIXEL_GRID_SPACING = std::floor(ROBOT_PIXEL_WIDTH);
const int HALF_PIXEL_GRID_SPACING = std::floor(ROBOT_PIXEL_WIDTH * 0.5);

/**
 * @brief Calculate the index for costmap data based on row and column position
 *
 * @param mx pixel column in the costmap
 * @param my pixel row in the costmap
 * @param width costmap width
 * @return costmap index
 */
inline unsigned int getIndex(unsigned int mx, unsigned int my, unsigned int width) {
    return my * width + mx;
}

/**
 * @brief Calculate the coverage plan
 *
 * @param costmap
 * @param polygon area to find coverage plan in
 * @param start_position the starting robot position to create a path from
 * @param[output] the output exploration path
 */
void getCoveragePlan(nav_msgs::msg::OccupancyGrid& costmap, std::vector<Location> polygon, Location start_position, std::vector<Location>& path);

/**
 * @brief Calculate the cost for a cell
 *
 * @param costmap
 * @param start_cell the starting cell in the costmap
 * @param end_cell the end cell in the costmap to find the cost to move to
 * @param previous_travel_angle the previous angle the robot traveled at
 * @return
 */
double costCalculator(const nav_msgs::msg::OccupancyGrid& costmap, Location start_cell, Location end_cell, double previous_travel_angle);

/**
 * @brief Generates a vector of free neighbors from the starting cell using BFS, if neighbors found the function returns
 *
 * @param map_temp costmap
 * @param polygon area which restricts where neighbors can be located
 * @param start_cell cell to start at in BFS
 * @param[output] free_neighbors vector
 * @return true if neighbors found
 * @return false otherwise
 */
bool generateFreeNeighborNodes(nav_msgs::msg::OccupancyGrid map_temp, std::vector<Location> polygon, Location start_cell,
                               std::vector<Location>& free_neighbors);

/**
 * @brief Get the cost of a cell in the costmap
 *
 * @param costmap
 * @param mx pixel column in the costmap
 * @param my pixel row in the costmap
 * @return
 */
int getCost(const nav_msgs::msg::OccupancyGrid& costmap, unsigned int mx, unsigned int my);

/**
 * @brief Set the cost of a cell in the costmap
 *
 * @param costmap
 * @param mx pixel column in the costmap
 * @param my pixel row in the costmap
 * @param cost
 */
void setCost(nav_msgs::msg::OccupancyGrid& costmap, unsigned int mx, unsigned int my, unsigned int cost);

/**
 * @brief Get the discretized costmap node center based on robot pixel width for a given cell
 *
 * @param costmap
 * @param mx pixel column in the costmap
 * @param my pixel row in the costmap
 * @return the center point in the discretized costmap for which the input indexes belong
 */
Location getNodeCenter(const nav_msgs::msg::OccupancyGrid& costmap, unsigned int mx, unsigned int my);

/**
 * @brief Set the discretized costmap node visited based on robot pixel width for a given cell
 *
 * @param costmap
 * @param mx pixel column in the costmap
 * @param my pixel row in the costmap
 * @param cost
 */
void setNodeVisitedCost(nav_msgs::msg::OccupancyGrid& costmap, unsigned int mx, unsigned int my, unsigned int cost);

/**
 * @brief Calculate the bounded area coverage based on discretized visited and unvisited costmap cells inside a polygon
 *
 * @param map_temp costmap
 * @param polygon the area to check bounded coverage in
 * @param start_position the starting robot position to check coverage around
 * @return
 */
double getBoundedAreaCoverage(nav_msgs::msg::OccupancyGrid map_temp, std::vector<Location> polygon, Location start_position);

/**
 * @brief Calculate an optimal planner initial position if possible for a given section and occupancy grid, based on a current start position
 *
 * @param costmap
 * @param section section to find optimal start in
 * @param start_position start position of robot
 * @param best_position best planner initial position
 * @return true if start found
 * @return otherwise
 */
bool getOptimalStartLocation(const nav_msgs::msg::OccupancyGrid& costmap, std::vector<Location> section, Location start_position,
                             Location& best_position);

/**
 * @brief Calculates the length of a vector between two points A and B
 *
 * @param ax Point A, x location
 * @param ay Point A, y location
 * @param bx Point B, x location
 * @param by Point B, y location
 * @return length of vector between two points
 */
double euclideanDistance(double ax, double ay, double bx, double by);
                             
/**
 * @brief determines whether a point is inside or outside a polygon, based upon
 * the number of edges a horizontal line would pass through from the robot location to infinity (to right).
 * If the number of edges passed through is odd, the robot is inside the polygon. If even, then the robot is
 * outside the polygon.
 *
 * @param points polygon points
 * @param robot_location point to be dealt with
 * @return true if inside polygon
 * @return false if outside polygon
 */
bool isPointInsidePolygon(std::vector<Location> points, Location robot_location);

}  // namespace planner