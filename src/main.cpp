#include "ros/ros.h"

#include "simulator.h"
#include "worldretrieve.h"
#include "types.h"

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   */
  ros::init(argc, argv, "prm_sim");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  //Lets create a shared pointer to the WorldInfoBuffer
  TWorldInfoBuffer buffer;

  std::shared_ptr<WorldRetrieve> wr(new WorldRetrieve(nh, buffer));
  std::shared_ptr<Simulator> sim(new Simulator(nh, buffer));

  std::thread t1(&WorldRetrieve::heartbeatThread, wr);
  std::thread t2(&Simulator::prmThread, sim);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  t1.join();
  t2.join();

  return 0;
}



