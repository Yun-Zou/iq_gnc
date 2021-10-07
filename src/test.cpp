#include <utility>
#include <gnc_functions.hpp>

float max_waypoint_dist = 2;

std::vector<gnc_api_waypoint> search_grid(geometry_msgs::Point search_origin, std::pair<double, double> size, double spacing) {

  std::vector<gnc_api_waypoint> waypointList;
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
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);

  while (nextWayPoint.y < bounding_box.y) {
    while (nextWayPoint.x < bounding_box.x) {
      nextWayPoint.x = nextWayPoint.x + std::min(2.0, bounding_box.x - nextWayPoint.x);
      waypointList.push_back(nextWayPoint);
    }

    nextWayPoint.y = nextWayPoint.y + std::min(spacing, bounding_box.y - nextWayPoint.y);
    waypointList.push_back(nextWayPoint);

    while (nextWayPoint.x > search_origin.x) {
      nextWayPoint.x = nextWayPoint.x - std::min(2.0, nextWayPoint.x - search_origin.x);
      waypointList.push_back(nextWayPoint);
    }

    nextWayPoint.y = nextWayPoint.y + std::min(spacing, bounding_box.y - nextWayPoint.y);
    waypointList.push_back(nextWayPoint);
  }  

  return waypointList;
}

std::vector<gnc_api_waypoint> setCircularWaypoint(geometry_msgs::Point centre, float radius, float alt,
                                                  float start = M_PI, float rotation = 2 * M_PI) {
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
    current_rotation = ceil((current_rotation + std::min(min_angle, rotation + start - current_rotation)) * 100) / 100;
    nextWayPoint.x = centre.x + cos(current_rotation) * radius;
    nextWayPoint.y = centre.y + sin(current_rotation) * radius;
    waypointList.push_back(nextWayPoint);
  }

  return waypointList;
};

// include API

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "monash_motion_node");
  ros::NodeHandle monash_motion_node;


  // specify some waypoints
  std::vector<gnc_api_waypoint> waypointList;

  geometry_msgs::Point search_origin;
  search_origin.x = 10;
  search_origin.y = 5;
  search_origin.z = 1;

  std::pair<double,double> size = {4,6};
  double spacing = 2;

  waypointList = search_grid(search_origin, size, spacing);
  waypointList = setCircularWaypoint(search_origin, 5.0, 10.0);
  // gnc_api_waypoint nextWayPoint;
  // nextWayPoint.x = 0;
  // nextWayPoint.y = 0;
  // nextWayPoint.z = 1;
  // nextWayPoint.psi = 0;
  // waypointList.push_back(nextWayPoint);
  // nextWayPoint.x = 1;
  // nextWayPoint.y = 0;
  // nextWayPoint.z = 1;
  // nextWayPoint.psi = -90;
  // waypointList.push_back(nextWayPoint);
  // nextWayPoint.x = 1;
  // nextWayPoint.y = 1;
  // nextWayPoint.z = 1;
  // nextWayPoint.psi = 0;
  // waypointList.push_back(nextWayPoint);
  // nextWayPoint.x = 0;
  // nextWayPoint.y = 1;
  // nextWayPoint.z = 1;
  // nextWayPoint.psi = 90;
  // waypointList.push_back(nextWayPoint);
  // nextWayPoint.x = 0;
  // nextWayPoint.y = 0;
  // nextWayPoint.z = 1;
  // nextWayPoint.psi = 180;
  // waypointList.push_back(nextWayPoint);
  // nextWayPoint.x = 0;
  // nextWayPoint.y = 0;
  // nextWayPoint.z = 1;
  // nextWayPoint.psi = 0;
  // waypointList.push_back(nextWayPoint);

  // specify control loop rate. We recommend a low frequency to not over load
  // the FCU with messages. Too many messages will cause the drone to be
  // sluggish
  ros::Rate rate(2.0);
  int counter = 0;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
      if (counter < waypointList.size()) {
        ROS_INFO("Waypoint %d: %f,%f,%f psi:%f", counter, waypointList[counter].x, waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
        counter++;
      } else {
        // land after all waypoints are reached
      }
  }
  return 0;
}

void follow(geometry_msgs::Point pose, geometry_msgs::Point destination, float alt) {

}

void set_home(geometry_msgs::Point pose, geometry_msgs::Point destination, float alt) {
  

}

void set_home(geometry_msgs::Point pose, geometry_msgs::Point destination, float alt) {

}