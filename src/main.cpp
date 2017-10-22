/*! @file
 *
 *  @brief Main program.
 *
 *  TODO
 *
 *  @author arosspope
 *  @date 11-09-2017
*/
/*!
 *  @addtogroup Main_module Main module documentation.
 *  @{
*/
/* MODULE main */
#include "ros/ros.h"

#include "simulator.h"
#include "worldretrieve.h"
#include "types.h"

#include <signal.h>

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   */
  ros::init(argc, argv, "prm_sim_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  std::vector<std::thread> threads;

  //Lets create a shared pointer to the WorldDataBuffer
  //It is populated by WorldRetrieve, and consumed by Simulator
  TWorldDataBuffer buffer;

  std::shared_ptr<WorldRetrieve> wr(new WorldRetrieve(nh, buffer));
  std::shared_ptr<Simulator> sim(new Simulator(nh, buffer));

  threads.push_back(std::thread(&WorldRetrieve::heartbeatThread, wr));
  threads.push_back(std::thread(&Simulator::plannerThread, sim));
  threads.push_back(std::thread(&Simulator::overlayThread, sim));

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the threads
   */
  ros::shutdown();

  //TODO: work out what to do when SIGINT is recieved in threads.

  //TODO: check correct data on /path topic.

  //TODO: Should I expand configuration space of unknown areas?
  //      If so, this creates the non-ideal case on startup when the
  //      is sitting in unknown space after expansion.

  //TODO: Moving the robot before requesting a goal seems to crash the simulator
  //      when a goal is requested... will need to investigate.

  //TODO: Heartbeat thread

  for(auto & t: threads){
    t.join();
  }



  return 0;
}
/*!
** @}
*/
