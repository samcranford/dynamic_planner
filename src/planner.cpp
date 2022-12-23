#include "planner.hpp"
#include <float.h>
#include <list>

namespace planner
{

    void getCoveragePlan(nav_msgs::msg::OccupancyGrid &costmap, std::vector<Location> polygon, Location start_position, std::vector<Location> &path)
    {
        // Set the start position as the inital coverage point
        Location start_node;
        int mx = start_position.x / costmap.info.resolution;
        int my = start_position.y / costmap.info.resolution;
        start_node.x = mx;
        start_node.y = my;
        setNodeVisitedCost(costmap, start_node.x, start_node.y, VISITED_OCCUPANCY);
        Location last_node = start_node;
        // Generate the coverage plan based on selecting the neighbor which minimizes cost
        std::vector<Location> coverage_path;
        coverage_path.push_back(start_node);
        Location next_node;
        std::vector<Location> free_neighbors;
        double previous_travel_angle = 0;
        while (generateFreeNeighborNodes(costmap, polygon, last_node, free_neighbors))
        {
            double min_cost = DBL_MAX;
            for (size_t i = 0; i < free_neighbors.size(); i++)
            {
                const double current_cost = costCalculator(costmap, last_node, free_neighbors[i], previous_travel_angle);
                if (current_cost < min_cost)
                {
                    min_cost = current_cost;
                    next_node = free_neighbors[i];
                }
            }
            // Append best neighbor to the coverage plan and update the travel angle
            previous_travel_angle = std::atan2(next_node.y - last_node.y, next_node.x - last_node.x);
            coverage_path.push_back(next_node);
            setNodeVisitedCost(costmap, int(next_node.x), int(next_node.y), VISITED_OCCUPANCY);
            last_node = next_node;
        }
        // Convert the coverage plan into map positions
        for (std::vector<Location>::iterator pose = coverage_path.begin(); pose != coverage_path.end(); ++pose)
        {
            Location current_pose;
            current_pose.x = pose->x * costmap.info.resolution; // costmap.info.origin.position.x; (If origin is not 0,0 add)
            current_pose.y = pose->y * costmap.info.resolution; // costmap.info.origin.position.y; (If origin is not 0,0 add)
            path.push_back(current_pose);
        }
        // Remove intermidiate points, based on if collinear
        if (path.size() > 1)
        {
            size_t path_size = path.size();
            for (size_t i = 1; i < path_size - 1; i++)
            {
                bool collinear =
                    (path[i].y - path[i - 1].y) * (path[i + 1].x - path[i].x) - (path[i].x - path[i - 1].x) * (path[i + 1].y - path[i].y) == 0;
                if (collinear)
                {
                    path.erase(path.begin() + i);
                    i--;
                    path_size--;
                    if (path_size <= 1)
                    {
                        return;
                    }
                }
            }
        }
        return;
    }

    double costCalculator(const nav_msgs::msg::OccupancyGrid &costmap, Location start_cell, Location end_cell, double previous_travel_angle)
    {
        // Distance, rotation, neighbor, and boundary weights
        double distance_weight = 1.0;
        double rotation_weight = 1.5;
        double neighbor_weight = 1.5;
        double boundary_weight = .5;

        // Calculate costs based on weights
        Location delta;
        delta.x = end_cell.x - start_cell.x;
        delta.y = end_cell.y - start_cell.y;
        double distance_cost = (sqrt(delta.x * delta.x + delta.y * delta.y) / PIXEL_GRID_SPACING) * distance_weight;

        double travel_angle = std::atan2(delta.y, delta.x);
        double delta_angle = std::abs(travel_angle - previous_travel_angle);
        delta_angle = delta_angle > 180 ? 360 - delta_angle : delta_angle;
        double rotation_cost = (delta_angle / 180) * rotation_weight;

        int visited_neighbors = 0;
        int boundary_points = 0;
        for (int i = end_cell.x - PIXEL_GRID_SPACING; i <= end_cell.x + PIXEL_GRID_SPACING; i += PIXEL_GRID_SPACING)
        {
            for (int j = end_cell.y - PIXEL_GRID_SPACING; j <= end_cell.y + PIXEL_GRID_SPACING; j += PIXEL_GRID_SPACING)
            {
                if (i < 0 || i >= int(costmap.info.width))
                {
                    continue;
                }
                if (j < 0 || j >= int(costmap.info.height))
                {
                    continue;
                }
                if (i == end_cell.x && j == end_cell.y)
                {
                    continue;
                }
                int cell_test = getCost(costmap, i, j);
                if (cell_test > VISITED_OCCUPANCY)
                {
                    boundary_points++;
                }
                else if (cell_test != FREE_OCCUPANCY)
                {
                    visited_neighbors++;
                }
            }
        }
        double neighbor_cost = (1.0 - visited_neighbors / 8.0) * neighbor_weight;
        double boundary_cost = (1.0 - boundary_points / 8.0) * boundary_weight;
        return distance_cost + rotation_cost + neighbor_cost + boundary_cost;
    }

    bool generateFreeNeighborNodes(nav_msgs::msg::OccupancyGrid map_temp, std::vector<Location> polygon, Location start_cell,
                                   std::vector<Location> &free_neighbors)
    {
        // Find neighbors
        free_neighbors.clear();
        Location last_node = start_cell;
        std::list<Location> queue;
        queue.emplace_back(last_node);
        while (!queue.empty())
        {
            Location node = queue.front();
            queue.pop_front();
            if (free_neighbors.size() > 0)
            {
                return true;
            }
            for (int i = node.x - PIXEL_GRID_SPACING; i <= node.x + PIXEL_GRID_SPACING; i += PIXEL_GRID_SPACING)
            {
                for (int j = node.y - PIXEL_GRID_SPACING; j <= node.y + PIXEL_GRID_SPACING; j += PIXEL_GRID_SPACING)
                {
                    if (i < 0 || i >= int(map_temp.info.width))
                    {
                        continue;
                    }
                    if (j < 0 || j >= int(map_temp.info.height))
                    {
                        continue;
                    }
                    if (i == node.x && j == node.y)
                    {
                        continue;
                    }

                    int cell_test = getCost(map_temp, i, j);
                    Location temp_position;
                    temp_position.x = i * map_temp.info.resolution;
                    temp_position.y = j * map_temp.info.resolution;
                    if (cell_test <= VISITED_OCCUPANCY && isPointInsidePolygon(polygon, temp_position))
                    {
                        Location temp_point;
                        temp_point.x = i;
                        temp_point.y = j;
                        if (cell_test == FREE_OCCUPANCY)
                        {
                            free_neighbors.emplace_back(temp_point);
                        }
                        setNodeVisitedCost(map_temp, i, j, FATAL_OCCUPANCY);
                        queue.emplace_back(temp_point);
                    }
                }
            }
        }
        return false;
    }

    int getCost(const nav_msgs::msg::OccupancyGrid &costmap, unsigned int mx, unsigned int my)
    {
        return costmap.data[getIndex(mx, my, costmap.info.width)];
    }

    void setCost(nav_msgs::msg::OccupancyGrid &costmap, unsigned int mx, unsigned int my, unsigned int cost)
    {
        costmap.data[getIndex(mx, my, costmap.info.width)] = cost;
    }

    Location getNodeCenter(const nav_msgs::msg::OccupancyGrid &costmap, unsigned int mx, unsigned int my)
    {
        int max_x = int(costmap.info.width - 1) > 0 ? int(costmap.info.width - 1) : 0;
        int max_y = int(costmap.info.height - 1) > 0 ? int(costmap.info.height - 1) : 0;
        int node_center_x = std::floor(mx / PIXEL_GRID_SPACING) * PIXEL_GRID_SPACING + HALF_PIXEL_GRID_SPACING;
        int node_center_y = std::floor(my / PIXEL_GRID_SPACING) * PIXEL_GRID_SPACING + HALF_PIXEL_GRID_SPACING;
        node_center_x = std::clamp(node_center_x, 0, max_x);
        node_center_y = std::clamp(node_center_y, 0, max_y);
        Location node_center;
        node_center.x = node_center_x;
        node_center.y = node_center_y;
        return node_center;
    }

    void setNodeVisitedCost(nav_msgs::msg::OccupancyGrid &costmap, unsigned int mx, unsigned int my, unsigned int cost)
    {
        int min_x = std::max(int(mx - HALF_PIXEL_GRID_SPACING), 0);
        int max_x = std::min(int(mx + HALF_PIXEL_GRID_SPACING), int(costmap.info.width));
        int min_y = std::max(int(my - HALF_PIXEL_GRID_SPACING), 0);
        int max_y = std::min(int(my + HALF_PIXEL_GRID_SPACING), int(costmap.info.height));
        for (int v = min_x; v < max_x; v++)
        {
            for (int u = min_y; u < max_y; u++)
            {
                costmap.data[getIndex(v, u, costmap.info.width)] = cost;
            }
        }
    }

    double getBoundedAreaCoverage(nav_msgs::msg::OccupancyGrid map_temp, std::vector<Location> polygon, Location start_position)
    {
        int mx = start_position.x / map_temp.info.resolution;
        int my = start_position.y / map_temp.info.resolution;
        Location last_node;
        last_node.x = mx;
        last_node.y = my;
        double total_travelable = 0;
        double visited = 0;
        std::list<Location> queue;
        queue.emplace_back(last_node);
        while (!queue.empty())
        {
            Location node = queue.front();
            queue.pop_front();
            for (int i = node.x - PIXEL_GRID_SPACING; i <= node.x + PIXEL_GRID_SPACING; i += PIXEL_GRID_SPACING)
            {
                for (int j = node.y - PIXEL_GRID_SPACING; j <= node.y + PIXEL_GRID_SPACING; j += PIXEL_GRID_SPACING)
                {
                    if (i < 0 || i >= int(map_temp.info.width))
                    {
                        continue;
                    }
                    if (j < 0 || j >= int(map_temp.info.height))
                    {
                        continue;
                    }
                    if (i == node.x && j == node.y)
                    {
                        continue;
                    }
                    int cell_test = getCost(map_temp, i, j);
                    Location temp_location;
                    temp_location.x = i * map_temp.info.resolution;
                    temp_location.y = j * map_temp.info.resolution;
                    if (cell_test <= VISITED_OCCUPANCY && isPointInsidePolygon(polygon, temp_location))
                    {
                        Location temp_point;
                        temp_point.x = i;
                        temp_point.y = j;
                        total_travelable++;
                        if (cell_test != FREE_OCCUPANCY)
                        {
                            visited++;
                        }
                        setNodeVisitedCost(map_temp, i, j, FATAL_OCCUPANCY);
                        queue.emplace_back(temp_point);
                    }
                }
            }
        }
        return total_travelable == 0 ? 100 : (visited / total_travelable) * 100;
    }

    bool getOptimalStartLocation(const nav_msgs::msg::OccupancyGrid &costmap, std::vector<Location> section, Location start_position,
                                 Location &best_position)
    {
        Location temp_best_position;
        double best_distance = DBL_MAX;
        bool best_found = false;

        // Check start position first to see if it is in section and has an available cost
        int index_start_x = start_position.x / costmap.info.resolution;
        int index_start_y = start_position.y / costmap.info.resolution;
        bool valid_indexes =
            index_start_x > 0 && index_start_x < int(costmap.info.width) && index_start_y > 0 && index_start_y < int(costmap.info.height);
        if (valid_indexes && isPointInsidePolygon(section, start_position))
        {
            if (getCost(costmap, index_start_x, index_start_y) <= VISITED_OCCUPANCY)
            {
                best_position = start_position;
                return true;
            }
        }

        // Find best based on proximity to the start position
        double min_x = section[0].x;
        double max_x = section[0].x;
        double min_y = section[0].y;
        double max_y = section[0].y;
        for (size_t i = 0; i < section.size(); i++)
        {
            min_x = section[i].x < min_x ? section[i].x : min_x;
            max_x = section[i].x > max_x ? section[i].x : max_x;
            min_y = section[i].y < min_y ? section[i].y : min_y;
            max_y = section[i].y > max_y ? section[i].y : max_y;
        }
        // Compute discretized max and min costmap indexes based on position max and mins and iterate through them at the defined robot pixel width
        int index_min_x = std::clamp(int(min_x / costmap.info.resolution), 0, int(costmap.info.width - 1));
        int index_max_x = std::clamp(int(max_x / costmap.info.resolution), 0, int(costmap.info.width - 1));
        int index_min_y = std::clamp(int(min_y / costmap.info.resolution), 0, int(costmap.info.height - 1));
        int index_max_y = std::clamp(int(max_y / costmap.info.resolution), 0, int(costmap.info.height - 1));
        Location costmap_point_min = getNodeCenter(costmap, index_min_x, index_min_y);
        Location costmap_point_max = getNodeCenter(costmap, index_max_x, index_max_y);

        for (double x = costmap_point_min.x; x < costmap_point_max.x; x += ROBOT_PIXEL_WIDTH)
        {
            for (double y = costmap_point_min.y; y < costmap_point_max.y; y += ROBOT_PIXEL_WIDTH)
            {
                Location position;
                position.x = x * costmap.info.resolution;
                position.y = y * costmap.info.resolution;
                if (isPointInsidePolygon(section, position))
                {
                    if (getCost(costmap, x, y) <= VISITED_OCCUPANCY)
                    {
                        double distance = euclideanDistance(position.x, position.y, start_position.x, start_position.y);
                        if (distance < best_distance)
                        {
                            best_found = true;
                            best_distance = distance;
                            temp_best_position = position;
                        }
                    }
                }
            }
        }
        if (best_found)
        {
            best_position = temp_best_position;
        }
        return best_found;
    }

    double euclideanDistance(double ax, double ay, double bx, double by)
    {
        double dx = ax - bx;
        double dy = ay - by;
        return std::pow(std::pow(dx, 2.0) + std::pow(dy, 2.0), 0.5);
    }

    bool isPointInsidePolygon(std::vector<Location> points, Location robot_location)
    {
        // Raycasting Algorithm to determine if inside or outside geofence
        double epsilon = 0.0000001; // Small epsilon to shift location if it falls on vertex point
        bool inside = false;
        Location lower_coord;
        Location upper_coord;

        for (size_t i = 0; i < points.size() - 1; i++)
        {
            auto current_coord = points[i];
            auto next_coord = points[i + 1];
            if (current_coord.y > next_coord.y)
            {
                lower_coord = next_coord;
                upper_coord = current_coord;
            }
            else
            {
                lower_coord = current_coord;
                upper_coord = next_coord;
            }
            if (robot_location.y == upper_coord.y || robot_location.y == lower_coord.y)
            {
                robot_location.y += epsilon;
            }
            if (robot_location.y < upper_coord.y && robot_location.y > lower_coord.y)
            {
                if (robot_location.x < std::min(lower_coord.x, upper_coord.x))
                {
                    inside = !inside;
                }
                else if (robot_location.x < std::max(lower_coord.x, upper_coord.x))
                {
                    double slope = (upper_coord.y - lower_coord.y) / (upper_coord.x - lower_coord.x);
                    double current_slope = (robot_location.y - lower_coord.y) / (robot_location.x - lower_coord.x);
                    if (current_slope > slope)
                    {
                        inside = !inside;
                    }
                }
            }
        }
        return inside;
    }

} // namespace planner
