#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#include "ros/duration.h"
#include <ros/ros.h>
#include <utility>

#include <gnc_functions.hpp>

/**
 * @brief Facing direction used for circles movements.
 *        Forward: Drone faces in direction of travel
 *        Inside: Drone faces the centre point circle
 *        Outside: Drone faces away from centre point of circle
 * 
 */
enum facing_direction {Forward, Inside, Outside};

/**
 * @brief Struct of params for the grid movements
 * 
 */
struct grid_params {
  double length_x;  // Length of grid in x (m)
  double length_y;  // Length of grid in y (m)
  double spacing;   // Distance between lines (m)
  double waypoint_dist; // Distance between waypoints (m)
};

/**
 * @brief Struct of params for circle movements
 * 
 */
struct circle_params {
  double radius;    //
  double altitude;
  double start_point;
  double rotation;
  facing_direction direction;
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

  /**
   * @brief Calculate distance between current position and waypoint
   * 
   * @param waypoint 
   * @return double 
   */
  double calc_waypoint_dist(gnc_api_waypoint &waypoint) {
    geometry_msgs::Point current_position = get_current_location();

    float deltaX = abs(waypoint.x - current_position.x);
    float deltaY = abs(waypoint.y - current_position.y);

    float dMag = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

    return dMag;
  }

  /**
   * @brief Convert angle so its between between -180 and 180 degrees
   * 
   * @param angle angle in degrees
   * @return float 
   */
  float wrap_angle(float angle) {
    return -180 + fmod(360 + fmod(angle - 180,360),360);
  }

  /**
   * @brief Turn the angle into drone heading. Drone heading considers "up" or 90 degrees as the zero angle.
   * 
   * @param angle angle in degrees
   * @return float 
   */
  float angle2heading(float angle) {
    float heading = angle - 90;

    return wrap_angle(heading);
  }

  float rad2deg(float radians) {
    return radians * 180 / M_PI;
  }

  float deg2rad(float degrees) {
    return degrees * M_PI / 180;
  }

  /**
   * @brief Turn gnc_api_waypoint Waypoint into geometry_msgs Point
   * 
   * @param waypoint 
   * @return geometry_msgs::Point 
   */
  geometry_msgs::Point waypoint2point(gnc_api_waypoint &waypoint) {
    geometry_msgs::Point point;
    point.x = waypoint.x;
    point.y = waypoint.y;
    point.z = waypoint.z;
    return point;
  }

  /**
   * @brief Get angle heading to point towards point2 from point1
   * 
   * @param point1 
   * @param point2 
   * @return float 
   */
  float get_direct_heading(geometry_msgs::Point point1, geometry_msgs::Point point2) {
    float deltaX = point2.x - point1.x;
    float deltaY = point2.y - point1.y;
    float psi = atan2(deltaY, deltaX);
    return angle2heading(rad2deg(psi));
  };

  /**
   * @brief Append waypoints to waypoint list of grid in shape determined by search_origin and params
   * 
   * @param waypointList List of waypoints
   * @param search_origin Starting point of grid left hand corner
   * @param params Grid parameters
   */
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

    // Go along x direction, then up, until y limit reached
    while (nextWayPoint.y < bounding_box.y) {

      // Continue along x direction until limit reached
      while (nextWayPoint.x < bounding_box.x) {
        nextWayPoint.x = nextWayPoint.x + std::min(params.waypoint_dist, bounding_box.x - nextWayPoint.x);
        nextWayPoint.psi = -90;
        waypointList.push_back(nextWayPoint);
      }
      
      // Increment in y direction
      nextWayPoint.y = nextWayPoint.y + std::min(params.spacing, bounding_box.y - nextWayPoint.y);
      nextWayPoint.psi = 0;
      waypointList.push_back(nextWayPoint);

      // Head back in other direction along x
      while (nextWayPoint.x > search_origin.x) {
        nextWayPoint.x = nextWayPoint.x - std::min((float) params.waypoint_dist, nextWayPoint.x - search_origin.x);
        nextWayPoint.psi = 90;
        waypointList.push_back(nextWayPoint);
      }

      // Increment in y direction
      nextWayPoint.y = nextWayPoint.y + std::min(params.spacing, bounding_box.y - nextWayPoint.y);
      nextWayPoint.psi = 0;
      waypointList.push_back(nextWayPoint);
    }
  }

  /**
   * @brief Append waypoints which move drone in circular motion
   * 
   * @param waypointList 
   * @param centre 
   * @param params
   * @return std::vector<gnc_api_waypoint> 
   */
  std::vector<gnc_api_waypoint> set_circular_waypoint( std::vector<gnc_api_waypoint> &waypointList, geometry_msgs::Point centre, circle_params params) {
   
    double min_arc = 1; // Minimum distance between points on arc
    double min_angle = min_arc / params.radius;

    gnc_api_waypoint nextWayPoint;

    double current_rotation = params.start_point;

    // Set first point in arc
    nextWayPoint.x = centre.x + cos(params.start_point) * params.radius;
    nextWayPoint.y = centre.y + sin(params.start_point) * params.radius;
    nextWayPoint.z = centre.z + params.altitude;
    nextWayPoint.psi = get_direct_heading(centre, waypoint2point(nextWayPoint));

    waypointList.push_back(nextWayPoint);

    // Until needed rotation has reached
    while (current_rotation < (params.rotation + params.start_point)) {
      gnc_api_waypoint oldWaypoint = nextWayPoint;

      // Go to next point in arc
      current_rotation = ceil((current_rotation + std::min(min_angle, params.rotation + params.start_point - current_rotation)) * 1000) / 1000;
      nextWayPoint.x = centre.x + cos(current_rotation) * params.radius;
      nextWayPoint.y = centre.y + sin(current_rotation) * params.radius;

      // Decide psi (heading) drone should have depending on travel_direction value
      float psi;
      if (params.direction == Forward) {
        psi = get_direct_heading(waypoint2point(oldWaypoint), waypoint2point(nextWayPoint) );
      } else if (params.direction == Outside) {
        psi = -get_direct_heading(waypoint2point(nextWayPoint), centre);
      } else if (params.direction == Inside) {
        psi = get_direct_heading(waypoint2point(nextWayPoint),centre);
      }

      nextWayPoint.psi = psi;
      
      waypointList.push_back(nextWayPoint);
    }

    return waypointList;
  };

  /**
   * @brief Append waypoint to waypoint list which tries to follow target. It will try to maintain a certain distance from target
   *        Aim to call this function when the drone is set to the end of the waypoint list as it uses current position
   * 
   * @param waypointList
   * @param target_pose
   * @param params 
   * @return true 
   * @return false 
   */
  bool set_follow(std::vector<gnc_api_waypoint> &waypointList, geometry_msgs::PoseStamped &target_pose, follow_params params) {

    geometry_msgs::Point current_position = get_current_location();
    gnc_api_waypoint nextWaypoint;

    // Target position
    gnc_api_waypoint target_waypoint;
    target_waypoint.x = target_pose.pose.position.x;
    target_waypoint.y = target_pose.pose.position.y;
    target_waypoint.z = target_pose.pose.position.z;

    // Get current heading
    float q0 = target_pose.pose.orientation.w;
    float q1 = target_pose.pose.orientation.x;
    float q2 = target_pose.pose.orientation.y;
    float q3 = target_pose.pose.orientation.z;
    target_waypoint.psi = atan2((2 * (q0 * q3 + q1 * q2)), (1 - 2 * (pow(q2, 2) + pow(q3, 2))));

    // Find the heading needed to head toward
    float psi = deg2rad(get_direct_heading(current_position, waypoint2point(target_waypoint)));

    
    // Set altitude to be designated altitude above target
    nextWaypoint.z = target_pose.pose.position.z + params.altitude;
    nextWaypoint.psi = psi;

    double dist = calc_waypoint_dist(target_waypoint);

    // If far away, head towards position incrementally
    if (dist > params.far_away_dist) {
      // set_speed(params.rapid_speed);
      nextWaypoint.x = current_position.x + std::min(dist, params.waypoint_dist) *
                                                cos(psi + (M_PI / 2));
      nextWaypoint.y = current_position.y + std::min(dist, params.waypoint_dist) *
                                                sin(psi + (M_PI / 2));

    } 
    // If outside outer radius or within inner radius, head towards optimal radius
    else if (dist > params.maintain_outer || dist < params.maintain_inner) {
      // set_speed(params.normal_speed);

      // Optimal radius is midway between outer radius and inner radius
      float optimum_range = (params.maintain_inner + params.maintain_outer) / 2;
      nextWaypoint.x = current_position.x +
                       std::min(dist - optimum_range, params.waypoint_dist) *
                           cos(psi + (M_PI / 2));
      nextWaypoint.y = current_position.y +
                       std::min(dist - optimum_range, params.waypoint_dist) *
                           sin(psi + (M_PI / 2));

    } 
    // If within optimal range, stay in current position
    else {
      // set_speed(params.normal_speed);
      nextWaypoint.x = current_position.x;
      nextWaypoint.y = current_position.y;
    }
    
    waypointList.push_back(nextWaypoint);

    return true;
  }

  // TODO: Avoid an area
  void set_avoid();

  /**
   * @brief Append waypoint which returns home
   * 
   * @param waypointList 
   * @param altitude altitude to be at
   */
  void return_home(std::vector<gnc_api_waypoint> &waypointList, float altitude) {

    geometry_msgs::Point current_position = get_current_location();
    gnc_api_waypoint point;

    float psi = get_direct_heading(current_position, waypoint2point(point));

    point.x = 0.0;
    point.y = 0.0;
    point.z = altitude;
    point.psi = psi;
    waypointList.push_back(point);
    
  }
};