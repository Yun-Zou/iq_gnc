#include <ros/ros.h>
#include <utility>

#include <gnc_functions.hpp>

enum state { Grounded, Hover, Flight, Search, Follow, RTL };

class FlightAlgorithm {
protected:

  // General Flight Parameters
  double rapid_speed;
  double normal_speed;
  double altitude;

  // Waypoint Parameters
  double max_waypoint_dist;
  double waypoint_radius;

  // Search Mode Parameters
  double grid_length_x;
  double grid_length_y;
  double grid_spacing;
  double circle_radius;

  // Follow Mode Parameters
  double follow_alt;
  double follow_maintain_inner;
  double follow_maintain_outer;
  double follow_far;
  double follow_time;

  // Safe Conditions
  double max_radius;
  double max_altitude;

  // Display Configs
  bool display_waypoints;
  bool display_path;

  state current_mission = Grounded;

  std::vector<gnc_api_waypoint> waypointList;

public:
  void init_params(ros::NodeHandle nh) {

    // General Flight Parameters
    nh.getParam("rapid_speed", rapid_speed);
    nh.getParam("normal_speed", normal_speed);
    nh.getParam("altitude", altitude);

    // Waypoint Parameters
    nh.getParam("max_waypoint_dist", max_waypoint_dist);
    nh.getParam("waypoint_radius", waypoint_radius);

    // Search Mode Parameters
    nh.getParam("grid_length_x", grid_length_x);
    nh.getParam("grid_length_y", grid_length_y);
    nh.getParam("grid_spacing", grid_spacing);
    nh.getParam("circle_radius", circle_radius);

    // Follow Mode Parameters
    nh.getParam("follow_alt", follow_alt);
    nh.getParam("follow_maintain_inner", follow_maintain_inner);
    nh.getParam("follow_maintain_outer", follow_maintain_outer);
    nh.getParam("follow_far", follow_far);
    nh.getParam("follow_time", follow_time);

    // Safe Conditions
    nh.getParam("max_radius", rapid_speed);
    nh.getParam("max_altitude", rapid_speed);

    // Display Configs
    nh.getParam("display_waypoints", normal_speed);
    nh.getParam("display_path", rapid_speed);
  };

  void set_flight_mode(state mode) { current_mission = mode; };

  state get_flight_mode();

  void follow(std::vector<gnc_api_waypoint> &waypointList,
              geometry_msgs::Point destination) {
    geometry_msgs::Point current_location = get_current_location();
  };

  void relative_move(gnc_api_waypoint relative_position) {

    geometry_msgs::Point current_location = get_current_location();
    float current_heading = get_current_heading();

    gnc_api_waypoint nextWayPoint;
    nextWayPoint.x = current_location.x + relative_position.x;
    nextWayPoint.y = current_location.y + relative_position.y;
    nextWayPoint.z = current_location.z + relative_position.z;
    nextWayPoint.psi = current_heading + relative_position.psi;

    waypointList.push_back(nextWayPoint);
  };

  void absolute_move(gnc_api_waypoint absolute_position) {

    gnc_api_waypoint nextWayPoint;
    nextWayPoint.x = absolute_position.x;
    nextWayPoint.y = absolute_position.y;
    nextWayPoint.z = absolute_position.z;
    nextWayPoint.psi = absolute_position.psi;

    waypointList.push_back(nextWayPoint);
  };

  void search_grid(gnc_api_waypoint search_origin, std::pair<double, double> size, double spacing) {

    gnc_api_waypoint nextWayPoint;

    geometry_msgs::Point bounding_box;
    bounding_box.x = search_origin.x + size.first;
    bounding_box.y = search_origin.y + size.second;
    bounding_box.z = search_origin.z;

    geometry_msgs::Point current_point;

    // Go to search start
    nextWayPoint.x = search_origin.x;
    nextWayPoint.y = search_origin.y;
    nextWayPoint.z = search_origin.z;
    nextWayPoint.psi = search_origin.psi;
    waypointList.push_back(nextWayPoint);

    while (nextWayPoint.y < bounding_box.y) {
      while (nextWayPoint.x < bounding_box.x) {
        nextWayPoint.x =
            nextWayPoint.x + std::min(2.0, bounding_box.x - nextWayPoint.x);
        waypointList.push_back(nextWayPoint);
      }

      nextWayPoint.y =
          nextWayPoint.y + std::min(spacing, bounding_box.y - nextWayPoint.y);
      waypointList.push_back(nextWayPoint);

      while (nextWayPoint.x > search_origin.x) {
        nextWayPoint.x = nextWayPoint.x - std::min(float (2.0), nextWayPoint.x - search_origin.x);
        waypointList.push_back(nextWayPoint);
      }

      nextWayPoint.y =
          nextWayPoint.y + std::min(spacing, bounding_box.y - nextWayPoint.y);
      waypointList.push_back(nextWayPoint);
    }
  }

  std::vector<gnc_api_waypoint> setCircularWaypoint(geometry_msgs::Point centre,
                                                    float radius, float alt,
                                                    float start = M_PI,
                                                    float rotation = 2 * M_PI) {
    float min_arc = 1;
    float min_angle = min_arc / radius;
    ROS_INFO("min_angle %f", min_angle);

    std::vector<gnc_api_waypoint> waypointList;
    gnc_api_waypoint nextWayPoint;

    float current_rotation = start;

    nextWayPoint.x = centre.x + cos(start) * radius;
    nextWayPoint.y = centre.y + sin(start) * radius;
    nextWayPoint.z = centre.z + alt;
    nextWayPoint.psi = 0;
    waypointList.push_back(nextWayPoint);

    while (current_rotation < (rotation + start)) {
      current_rotation = ceil((current_rotation +std::min(min_angle, rotation + start - current_rotation)) * 100) / 100;
      nextWayPoint.x = centre.x + cos(current_rotation) * radius;
      nextWayPoint.y = centre.y + sin(current_rotation) * radius;
      waypointList.push_back(nextWayPoint);
    }

    return waypointList;
  };

  std::vector<gnc_api_waypoint> getWayponts() { return waypointList; };

  void clearWaypoints() { waypointList.clear(); };

  void follow();

  void flight_path();

  void goHome();

  void avoid();

  void stop();

  void subscribe();

  void publish();

  void logging();

  void check_safety();
};