#include "../FlightController.cpp"
// include API

static FlightController flight_controller;

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "testing_function");
  ros::NodeHandle testing_function("~");

  // initialize control publisher/subscribers
  flight_controller.init(testing_function);
  double alt = flight_controller.get_normal_altitude();
  // specify some waypoints
  flight_controller.absolute_move_WP(0, 0, alt, 0);
  flight_controller.absolute_move_WP(2, 0, alt, -90);
  flight_controller.absolute_move_WP(2, 5, alt, 0);
  flight_controller.absolute_move_WP(-3, 5, alt, 90);
  flight_controller.set_flight_mode(Search);

  std::vector<gnc_api_waypoint> waypointList = flight_controller.get_waypoints();
  state current_mode = flight_controller.get_flight_mode();
  
  // specify control loop rate. We recommend a low frequency to not over load
  // the FCU with messages. Too many messages will cause the drone to be
  // sluggish
  ros::Rate rate(2.0);
  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();

    waypointList = flight_controller.get_waypoints();
    current_mode = flight_controller.get_flight_mode();

    int counter = flight_controller.counter;

    // Land if UAV has reached home position
    if (current_mode == RTL || current_mode == Land) {
      flight_controller.set_flight_mode(Land);

    } 

    // Go to each waypoint
    else if (counter < waypointList.size()) {
      // set_destination(waypointList[counter].x, waypointList[counter].y, 
      //                 waypointList[counter].z, waypointList[counter].psi);
      ROS_INFO("waypoint %d: %f, %f, %f, %f", counter, waypointList[counter].x, waypointList[counter].y,
                      waypointList[counter].z, waypointList[counter].psi);

      flight_controller.counter++;

    } 

    // Go home if no remaining commands
    else {
      // return home after all waypoints are reached
      flight_controller.set_flight_mode(RTL);
    }
  
    flight_controller.publish_topics();
  }

  return 0;
}