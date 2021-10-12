#include "FlightController.cpp"
// include API

static FlightController flight_controller;

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "monash_motion_node");
  ros::NodeHandle monash_motion_node("~");

  // initialize control publisher/subscribers
  init_publisher_subscriber(monash_motion_node);

  // init params
  flight_controller.init_params();

  // wait for FCU connection
  wait4connect();

  // wait for used to switch to mode GUIDED
  wait4start();

  // create local reference frame
  initialize_local_frame();

  set_speed(flight_controller.get_normal_speed());

  // request takeoff
  takeoff(2);

  flight_controller.set_flight_mode(Flight);

  // specify some waypoints
  gnc_api_waypoint absolute_move;

  flight_controller.absolute_move_WP(0,0,2,0);
  flight_controller.absolute_move_WP(2,0,2,-90);
  flight_controller.absolute_move_WP(2,2,2,0);

  std::vector<gnc_api_waypoint> waypointList = flight_controller.get_waypoints();

  // specify control loop rate. We recommend a low frequency to not over load
  // the FCU with messages. Too many messages will cause the drone to be
  // sluggish
  ros::Rate rate(2.0);
  int counter = 0;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    if (check_waypoint_reached(.3) == 1) {
      if (counter < waypointList.size()) {
        set_destination(waypointList[counter].x, waypointList[counter].y,
                        waypointList[counter].z, waypointList[counter].psi);
        counter++;
      } else {
        // land after all waypoints are reached
        land();
        flight_controller.set_flight_mode(Grounded);
      }
    }
  }
  return 0;
}