/*! @file
 *
 *  @brief Entry point for the prm_sim_node.
 *
 *  A probabilistic roadmap planner is a motion-planning algorithm in robotics,
 *  which solves the problem of determining a path between a starting and a goal
 *  configuration of the robot while avoiding collisions.
 *
 *  The basic idea behind PRM is to take random samples from the configuration
 *  space of the robot, checking if they are in free space, then attempting to
 *  connect configurations (groups of samples) to other nearby configurations.
 *
 *  When starting the prm_sim_node (using rosrun), the following parameters may
 *  be specified.
 *
 *  - _map_size:=<size of supplied ogMap in meters>
 *  - _resolution:=<resolution of the opencv map image>
 *  - _density:=<max density the prm network can have>
 *  - _robot_diameter:=<the diameter of the robot in meters>
 *
 *  @author arosspope
 *  @date 23-10-2017
*/
#include "ros/ros.h"

#include "simulator.h"
#include "worldretrieve.h"
#include "types.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "prm_sim_node");

  //NodeHandle is the main access point to communications with the ROS system.
  ros::NodeHandle nh;

  //The WorldDataBuffer is populated by WorldRetrieve, and consumed by Simulator
  TWorldDataBuffer buffer;

  std::vector<std::thread> threads;
  std::shared_ptr<WorldRetrieve> wr(new WorldRetrieve(nh, buffer));
  std::shared_ptr<Simulator> sim(new Simulator(nh, buffer));

  threads.push_back(std::thread(&Simulator::plannerThread, sim));
  threads.push_back(std::thread(&Simulator::overlayThread, sim));


  //ros::spin() will enter a loop, pumping callbacks.  With this version, all
  //callbacks will be called from within this thread (the main one).
  //ros::spin() will exit when Ctrl-C is pressed, or the node is shutdown by the master.
  ros::spin();

  //Let's cleanup everything, shutdown ros and join the threads
  ros::shutdown();

  //Join threads and begin!
  for(auto & t: threads){
    t.join();
  }

  return 0;
}
