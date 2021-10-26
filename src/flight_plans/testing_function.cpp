#include "../FlightController.cpp"
// include API

void init_fake_initial_pos() {
  current_pose_g.pose.pose.position.x = 0;
  current_pose_g.pose.pose.position.y = 0;
  current_pose_g.pose.pose.position.z = 0;
  current_pose_g.pose.pose.orientation.x = 0;
  current_pose_g.pose.pose.orientation.y = 0;
  current_pose_g.pose.pose.orientation.z = 0;
  current_pose_g.pose.pose.orientation.w = 1;
}

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "testing_function");
  ros::NodeHandle testing_function("~");

  FlightController flight_controller(testing_function);

  // initialize control publisher/subscribers
  double alt = flight_controller.get_normal_altitude();

  init_fake_initial_pos();

  flight_controller.set_accepting_commands(true);

  // specify some waypoints
  flight_controller.absolute_move_WP(0, 0, alt, 0);
  flight_controller.absolute_move_WP(2, 0, alt, -90);
  flight_controller.absolute_move_WP(2, 5, alt, 0);
  flight_controller.absolute_move_WP(-3, 5, alt, 90);
  // flight_controller.set_flight_mode(Search);

  std::vector<gnc_api_waypoint> waypointList = flight_controller.get_waypoints();
  state current_mode = flight_controller.get_flight_mode();
  
    // flight_controller.set_flight_mode(Follow);

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
    if (flight_controller.check_target_validity()) {
      flight_controller.follow_target();
    }

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
    // else {
    //   // return home after all waypoints are reached
    //   flight_controller.set_flight_mode(RTL);
    // }
  
    flight_controller.publish_topics();
  }

  return 0;
}