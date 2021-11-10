/**
 * MIT License
 *
 * Copyright (c) 2021 Pratik Bhujnbal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the  
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE. 
 * 
 * @file listener.cpp
 * @author Pratik Bhujnbal
 * @brief  publisher ("talker") node which will continually broadcast a message
 * @version 1.0
 * @date 11/07/2021
 * @copyright  Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "beginner_tutorials/Str.h"

//  Initialize value to a string to publish
extern std::string pub_msg = "Hello from Patrick";
/**
 * @brief A service Fucntion for changing the output string of the publisher.
 * @param req service request
 * @param res service response
 * @return returns true when string changes
 */
bool SetString(beginner_tutorials::Str::Request &req,
               beginner_tutorials::Str::Response &res)
{
  res.output = req.data;
  pub_msg = req.data;
  ROS_DEBUG_STREAM("Initial String changed to : " << req.data);
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS sys tem.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.simple_rostest
   */
  ros::init(argc, argv, "talker");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // To create service and advertise over ROS.
  ros::ServiceServer service = n.advertiseService("SrvChgStr", &SetString);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // Setting default frequency for 20 Hz
  int frequency = 20;
  if (argc > 1)
  {
    frequency = atoi(argv[1]);
  }
  // Stream information once about frequency set from argument
  ROS_INFO_STREAM_ONCE("Set frequency= " << frequency);
  if (frequency <= 0)
  {
    // If frequency is negative or zero- Fatal
    ROS_FATAL_STREAM("Frequency must be greater than zero !");
    ROS_DEBUG_STREAM("Changing to default value i.e. 20 Hz");
    frequency = 20;
  }
  else if (frequency > 100)
  {
    // If frequency is greater than expected- ERROR
    ROS_ERROR_STREAM("Expected frequency is less than 100 Hz");
    ROS_DEBUG_STREAM("Changing to default value i.e. 20 Hz");
    frequency = 20;
  }
  else if (0 < frequency && frequency < 5)
  {
    // If frequency is less than 5 and greater than 0- WARN
    ROS_WARN_STREAM("Input Frequency too low");
    ROS_DEBUG_STREAM("Changing to default value i.e. 20 Hz");
    frequency = 20;
  }
  ros::Rate loop_rate(frequency);

  // Creates a TransformBroadcaster object
  static tf::TransformBroadcaster pb;
  // Creates a Transform object
  tf::Transform transform;
  // Creates a Quaternion object
  tf::Quaternion quat;
  while (ros::ok())
  {

    // Send the transformations
    transform.setOrigin(tf::Vector3(1, 4, 3));
    quat.setRPY(0, 0, 90);
    transform.setRotation(quat);
    pb.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    ss << "ROS says " << pub_msg;
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
