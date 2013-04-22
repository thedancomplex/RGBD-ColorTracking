/*------------------------------------------------------------------------------
 *  Title:        talker.cpp
 *  Description:  ROS node example talker.
 *----------------------------------------------------------------------------*/

/*
 *
 *      Copyright (c) 2010 <iBotics -- www.sdibotics.org>
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the Stingray, iBotics nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "node_example_core.h"

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  NodeExample *node_example = new NodeExample();

  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server
  // values can be overwritten.
  dynamic_reconfigure::Server<node_example::node_example_paramsConfig> dr_srv;
  dynamic_reconfigure::Server<node_example::node_example_paramsConfig>::CallbackType cb;
  cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
  dr_srv.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  int a;
  int b;
  string message;
  int rate;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("a", a, int(1));
  private_node_handle_.param("b", b, int(2));
  private_node_handle_.param("message", message, string("hello"));
  private_node_handle_.param("rate", rate, int(40));

  // Create a publisher and name the topic.
  ros::Publisher pub_message = n.advertise<node_example::node_example_data>("example", 10);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (n.ok())
  {
    // Publish the message.
    node_example->publishMessage(&pub_message);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
