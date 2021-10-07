#include <utility>
#include <gnc_functions.hpp>


// include API

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "monash_motion_node");
  ros::NodeHandle monash_motion_node;

  // initialize control publisher/subscribers
  init_publisher_subscriber(monash_motion_node);

  // wait for FCU connection
  wait4connect();

  // wait for used to switch to mode GUIDED
  wait4start();

  // create local reference frame
  initialize_local_frame();

  // request takeoff
  takeoff(1);

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
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    if (check_waypoint_reached(.3) == 1) {
      waypointList.erase(waypointList.begin());

      if (!waypointList.empty()) {
        set_destination(waypointList[0].x, waypointList[0].y, waypointList[0].z, waypointList[0].psi);
      } else {
        // land after all waypoints are reached
        land();
      }
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