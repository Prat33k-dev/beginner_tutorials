/**
 * MIT License
 *
 * Copyright (c) 2021 Pratik Bhujbal
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
 * @file service_test
 * @author Pratik Bhujbal
 * @brief  cpp test file for service
 * @version 1.0
 * @date 11/07/2021
 * @copyright  Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/Str.h"

std::shared_ptr<ros::NodeHandle> nh;

/*
 * @brief: This test checks for service exisst or not
 * @return: none
 */
TEST(TESTSuite, serviceCheck)
{
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials::Str>(
      "SrvChgStr");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

/*
 * @brief: This test checks for service where string has changed or not
 * @return: none
 */
TEST(TESTSuite, serviceOutput)
{
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials::Str>(
      "SrvChgStr");
  beginner_tutorials::Str srv;
  srv.request.data = "pratik";
  client.call(srv);
  EXPECT_EQ("pratik", srv.response.output);
}

int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "service_test_node");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
