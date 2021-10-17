#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include "ros/duration.h"
#include <ros/ros.h>
#include <utility>

#include <gnc_functions.hpp>

struct grid_params {
  double length_x;
  double length_y;
  double spacing;
  double waypoint_dist;
};

struct circle_params {
  double radius;
  double altitude;
  double start_point;
  double rotation;
};

struct follow_params {
  double altitude;
  double maintain_inner;
  double maintain_outer;
  double far_away_dist;
  double follow_time;
  double waypoint_dist;
  double rapid_speed;
  double normal_speed;
};

class FlightAlgorithm {
protected:

public:

  double calc_waypoint_dist(gnc_api_waypoint &waypoint) {
    geometry_msgs::Point current_position = get_current_location();

    float deltaX = abs(waypoint.x - current_position.x);
    float deltaY = abs(waypoint.y - current_position.y);

    float dMag = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

    return dMag;
  }

  float wrap_angle(float angle) {
    angle = fmod(angle + 180, 360);

    if (angle < 0) {
      angle += 360;
    };
    angle -= -180;
    return angle;
  }

  void set_search_grid(std::vector<gnc_api_waypoint> &waypointList, gnc_api_waypoint &search_origin, grid_params params) {

    gnc_api_waypoint nextWayPoint;

    geometry_msgs::Point bounding_box;
    bounding_box.x = search_origin.x + params.length_x;
    bounding_box.y = search_origin.y + params.length_y;
    bounding_box.z = search_origin.z;

    // Go to search start
    nextWayPoint.x = search_origin.x;
    nextWayPoint.y = search_origin.y;
    nextWayPoint.z = search_origin.z;
    nextWayPoint.psi = search_origin.psi;
    waypointList.push_back(nextWayPoint);

    while (nextWayPoint.y < bounding_box.y) {
      while (nextWayPoint.x < bounding_box.x) {
        nextWayPoint.x = nextWayPoint.x + std::min(params.waypoint_dist, bounding_box.x - nextWayPoint.x);
        nextWayPoint.psi = -90;
        waypointList.push_back(nextWayPoint);
      }

      nextWayPoint.y = nextWayPoint.y + std::min(params.spacing, bounding_box.y - nextWayPoint.y);
      nextWayPoint.psi = 0;
      waypointList.push_back(nextWayPoint);

      while (nextWayPoint.x > search_origin.x) {
        nextWayPoint.x = nextWayPoint.x - std::min((float) params.waypoint_dist, nextWayPoint.x - search_origin.x);
        nextWayPoint.psi = 90;
        waypointList.push_back(nextWayPoint);
      }

      nextWayPoint.y = nextWayPoint.y + std::min(params.spacing, bounding_box.y - nextWayPoint.y);
      nextWayPoint.psi = 0;
      waypointList.push_back(nextWayPoint);
    }
  }

  std::vector<gnc_api_waypoint> set_circular_waypoint( std::vector<gnc_api_waypoint> &waypointList, geometry_msgs::Point centre, circle_params params) {
    double min_arc = 1;
    double min_angle = min_arc / params.radius;

    gnc_api_waypoint nextWayPoint;

    double current_rotation = params.start_point;

    nextWayPoint.x = centre.x + cos(params.start_point) * params.radius;
    nextWayPoint.y = centre.y + sin(params.start_point) * params.radius;
    nextWayPoint.z = centre.z + params.altitude;
    nextWayPoint.psi = 0;
    waypointList.push_back(nextWayPoint);

    while (current_rotation < (params.rotation + params.start_point)) {
      current_rotation = ceil((current_rotation + std::min(min_angle, params.rotation + params.start_point - current_rotation)) * 1000) / 1000;
      nextWayPoint.x = centre.x + cos(current_rotation) * params.radius;
      nextWayPoint.y = centre.y + sin(current_rotation) * params.radius;
      waypointList.push_back(nextWayPoint);
    }

    return waypointList;
  };

  bool set_follow(std::vector<gnc_api_waypoint> &waypointList, geometry_msgs::PoseStamped &target_pose, follow_params params) {

    geometry_msgs::Point current_position = get_current_location();
    gnc_api_waypoint nextWaypoint;

    gnc_api_waypoint target_waypoint;
    target_waypoint.x = target_pose.pose.position.x;
    target_waypoint.y = target_pose.pose.position.y;
    target_waypoint.z = target_pose.pose.position.z;

    float q0 = target_pose.pose.orientation.w;
    float q1 = target_pose.pose.orientation.x;
    float q2 = target_pose.pose.orientation.y;
    float q3 = target_pose.pose.orientation.z;
    target_waypoint.psi = atan2((2 * (q0 * q3 + q1 * q2)), (1 - 2 * (pow(q2, 2) + pow(q3, 2))));

    float deltaX = target_pose.pose.position.x - current_position.x;
    float deltaY = target_pose.pose.position.y - current_position.y;
    float psi = atan2(deltaY, deltaX) - M_PI / 2;
    psi = wrap_angle(psi);

    double dist = calc_waypoint_dist(target_waypoint);

    nextWaypoint.z = target_pose.pose.position.z + params.altitude;
    nextWaypoint.psi = psi;

    if (dist > params.far_away_dist) {
      set_speed(params.rapid_speed);
      nextWaypoint.x = current_position.x + std::min(dist, params.waypoint_dist) *
                                                cos(psi + (M_PI / 2));
      nextWaypoint.y = current_position.y + std::min(dist, params.waypoint_dist) *
                                                sin(psi + (M_PI / 2));

    } else if (dist > params.maintain_outer || dist < params.maintain_inner) {
      set_speed(params.normal_speed);
      float optimum_range = (params.maintain_inner + params.maintain_outer) / 2;
      nextWaypoint.x = current_position.x +
                       std::min(dist - optimum_range, params.waypoint_dist) *
                           cos(psi + (M_PI / 2));
      nextWaypoint.y = current_position.y +
                       std::min(dist - optimum_range, params.waypoint_dist) *
                           sin(psi + (M_PI / 2));

    } else {
      set_speed(params.normal_speed);
      nextWaypoint.x = current_position.x;
      nextWaypoint.y = current_position.y;
    }

    waypointList.push_back(nextWaypoint);

    return true;
  }

  void set_avoid();

  void return_home(std::vector<gnc_api_waypoint> &waypointList, float altitude) {

    geometry_msgs::Point current_position = get_current_location();
    gnc_api_waypoint point;

    float deltaX = 0 - current_position.x;
    float deltaY = 0 - current_position.y;
    float psi = atan2(deltaY, deltaX) - M_PI / 2;
    psi = wrap_angle(psi);

    point.x = 0.0;
    point.y = 0.0;
    point.z = altitude;
    point.psi = psi;
    waypointList.push_back(point);
    
  }
};