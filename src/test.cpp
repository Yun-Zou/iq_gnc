#include <utility>
#include <gnc_functions.hpp>


// include API

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "monash_motion_node");
  ros::NodeHandle monash_motion_node;


  // specify some waypoints
  std::vector<gnc_api_waypoint> waypointList;
  gnc_api_waypoint nextWayPoint;
  nextWayPoint.x = 0;
  nextWayPoint.y = 0;
  nextWayPoint.z = 1;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 1;
  nextWayPoint.y = 0;
  nextWayPoint.z = 1;
  nextWayPoint.psi = -90;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 1;
  nextWayPoint.y = 1;
  nextWayPoint.z = 1;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 0;
  nextWayPoint.y = 1;
  nextWayPoint.z = 1;
  nextWayPoint.psi = 90;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 0;
  nextWayPoint.y = 0;
  nextWayPoint.z = 1;
  nextWayPoint.psi = 180;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 0;
  nextWayPoint.y = 0;
  nextWayPoint.z = 1;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);

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

void search_grid(geometry_msgs::Point search_origin, std::pair <double,double> size, double spacing) {

}

void setCircularWaypoint() {

};

void follow() {

}

void clearWaypoints() {

}

void rapid_linear_move() {

}