#include "../FlightController.cpp"
// include API

static FlightController flight_controller;

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "monash_motion_node");
  ros::NodeHandle monash_motion_node("~");

  // initialize control publisher/subscribers
  init_publisher_subscriber(monash_motion_node);
  flight_controller.init();

  // wait for FCU connection
  wait4connect();

  // wait for used to switch to mode GUIDED
  wait4start();

  // create local reference frame
  initialize_local_frame();
  
  set_speed(flight_controller.get_normal_speed());

  // request takeoff
  // takeoff(flight_controller.get_normal_altitude());
  takeoff(1.0);

  flight_controller.set_flight_mode(Flight);

  // specify some waypoints
  // flight_controller.absolute_move_WP(0,0,2,0);
  flight_controller.absolute_move_WP(1,0,1,-90);
  flight_controller.absolute_move_WP(1,1,1,0);
  flight_controller.absolute_move_WP(0,1,1,90);
  flight_controller.absolute_move_WP(0,0,1,180);
  flight_controller.absolute_move_WP(0,0,1,0);

  std::vector<gnc_api_waypoint> waypointList = flight_controller.get_waypoints();
  state current_mode = flight_controller.get_flight_mode();

  // specify control loop rate. We recommend a low frequency to not over load
  // the FCU with messages. Too many messages will cause the drone to be
  // sluggish
  ros::Rate rate(2.0);
  int counter = flight_controller.waypoint_counter;

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    waypointList = flight_controller.get_waypoints();
    current_mode = flight_controller.get_flight_mode();

    if (check_waypoint_reached(flight_controller.get_waypoint_radius()) == 1) {
      
      if (current_mode == Land) {
        land();
      }

      else if (current_mode == RTL) {
        flight_controller.set_flight_mode(Land);
      }

      else if (counter < waypointList.size()) {
        set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
        counter++;
      }

      else {
        // return home after all waypoints are reached
        flight_controller.set_flight_mode(RTL);
      }
    }
  }
  return 0;
}