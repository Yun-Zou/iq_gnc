#include "geometry_msgs/Point.h"
#include "ros/duration.h"
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
  double circle_radius;
  double operation_time;

  // Waypoint Parameters
  double max_waypoint_dist;
  double waypoint_radius;

  // Search Mode Parameters
  double grid_length_x;
  double grid_length_y;
  double grid_spacing;
  double search_circle_radius;
  double search_lost_time;

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

  ros::Subscriber target_sub;
  nav_msgs::Odometry target_pose;
  ros::Time target_last_seen;
  bool target_valid = false;

  ros::Time begin;

  state current_mission = Grounded;

  std::vector<gnc_api_waypoint> waypointList;

public:
  void init_params(ros::NodeHandle nh) {

    // General Flight Parameters
    nh.getParam("rapid_speed", rapid_speed);
    nh.getParam("normal_speed", normal_speed);
    nh.getParam("circle_radius", circle_radius);
    nh.getParam("altitude", altitude);
    nh.getParam("operation_time", operation_time);

        // Waypoint Parameters
    nh.getParam("max_waypoint_dist", max_waypoint_dist);
    nh.getParam("waypoint_radius", waypoint_radius);

    // Search Mode Parameters
    nh.getParam("grid_length_x", grid_length_x);
    nh.getParam("grid_length_y", grid_length_y);
    nh.getParam("grid_spacing", grid_spacing);
    nh.getParam("search_circle_radius", search_circle_radius);
    nh.getParam("search_lost_time", search_lost_time);

    // Follow Mode Parameters
    nh.getParam("follow_alt", follow_alt);
    nh.getParam("follow_maintain_inner", follow_maintain_inner);
    nh.getParam("follow_maintain_outer", follow_maintain_outer);
    nh.getParam("follow_far", follow_far);
    nh.getParam("follow_time", follow_time);

    // Safe Conditions
    nh.getParam("max_radius", max_radius);
    nh.getParam("max_altitude", max_altitude);

    // Display Configs
    nh.getParam("display_waypoints", display_waypoints);
    nh.getParam("display_path", display_path);
  };

  void set_flight_mode(state mode) { current_mission = mode; };

  state get_flight_mode();

  void follow(std::vector<gnc_api_waypoint> &waypointList,
              geometry_msgs::Point destination) {
    geometry_msgs::Point current_location = get_current_location();
  };

  void absolute_move(gnc_api_waypoint absolute_position) {

    gnc_api_waypoint nextWayPoint;
    nextWayPoint.x = absolute_position.x;
    nextWayPoint.y = absolute_position.y;
    nextWayPoint.z = absolute_position.z;
    nextWayPoint.psi = absolute_position.psi;

    waypointList.push_back(nextWayPoint);
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
        nextWayPoint.x = nextWayPoint.x + std::min(2.0, bounding_box.x - nextWayPoint.x);
        nextWayPoint.psi = -90;
        waypointList.push_back(nextWayPoint);
      }

      nextWayPoint.y = nextWayPoint.y + std::min(spacing, bounding_box.y - nextWayPoint.y);
      nextWayPoint.psi = 0;
      waypointList.push_back(nextWayPoint);

      while (nextWayPoint.x > search_origin.x) {
        nextWayPoint.x = nextWayPoint.x - std::min(float (2.0), nextWayPoint.x - search_origin.x);
        nextWayPoint.psi = 90;
        waypointList.push_back(nextWayPoint);
      }

      nextWayPoint.y = nextWayPoint.y + std::min(spacing, bounding_box.y - nextWayPoint.y);
      nextWayPoint.psi = 0;
      waypointList.push_back(nextWayPoint);
    }
  }

  std::vector<gnc_api_waypoint> setCircularWaypoint(geometry_msgs::Point centre, float radius, float alt,
                                                    float start = M_PI, float rotation = 2 * M_PI) {
    float min_arc = 1;
    float min_angle = min_arc / radius;

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

  double get_rapid_speed() {
    return rapid_speed;
  }

  double get_normal_speed() {
    return normal_speed;
  }

  double get_normal_altitude() {
    return altitude;
  }

  double get_waypoint_radius() {
    return waypoint_radius;
  }

  double calc_waypoint_dist(gnc_api_waypoint waypoint) {
    geometry_msgs::Point current_position = get_current_location();

    float deltaX = abs(waypoint.x - current_position.x);
    float deltaY = abs(waypoint.y - current_position.y);

    float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2));

    return dMag;
  }

  void startTimer() {
    begin = ros::Time::now();
  };

  void check_target_validity() {
    ros::Duration time_diff = ros::Time::now() - target_last_seen;
    target_valid = !(time_diff > ros::Duration(search_lost_time));

    return;
  }

  void follow_target() {
    geometry_msgs::Point current_position = get_current_location();
    // geometry_msgs::Pose target = target_pose.pose.pose.position;

    // set_flight_mode(Follow);
    
    // double heading;
    // double psi;

    // if (dist > high) {
    //   // fly fast towards target
    // } else if (outside radius or inside radius) {
    //   // fly normal towards middle of inner ring
    // } else if (inside radius) {
    //   // point towards target
    // }

    return;
  }

  void go_home() {

    geometry_msgs::Point current_position = get_current_location();

    float deltaX = 0 - current_position.x;
    float deltaY = 0 - current_position.y;
    float psi = atan2(deltaY, deltaX) - M_PI/2;
    psi = fmod(psi + 180,360);

    if (psi < 0) {psi += 360;};
    psi -= -180;

    set_flight_mode(RTL);
    set_destination(0, 0, altitude, psi);

  }

  void flight_path();

  void avoid();

  void target_cb(const nav_msgs::Odometry &msg){
    target_last_seen = ros::Time::now();
    target_pose = msg;
  };

  void init_publisher_subscriber(ros::NodeHandle nh) {

    std::string ros_namespace;
    if (!nh.hasParam("namespace")) {
      ROS_INFO("using default namespace");
    } else {
      nh.getParam("namespace", ros_namespace);
      ROS_INFO("using namespace %s", ros_namespace.c_str());
    }

    // target_sub = nh.subscribe<nav_msgs::Odometry>((ros_namespace + "/monash_perception/target").c_str(), 5, target_cb);
  };

  void logging();

  void check_safety_conditions() {

    geometry_msgs::Point current_position = get_current_location();

    bool time_check = (ros::Time::now() - begin) > ros::Duration(operation_time);
    bool radius_check = pow(pow(current_position.x,2) + pow(current_position.y,2),1/2) > max_radius;
    bool altitude_check = current_position.z > max_altitude;

    if (time_check || radius_check || altitude_check) {
      set_flight_mode(RTL);
    }

    return;
  };
};