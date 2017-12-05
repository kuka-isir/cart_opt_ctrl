// read_sensor - ISIR Wed 06 Sep 2017 02:07:19 PM CEST
// Copyright (c) Lucas Joseph, All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3.0 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library.

#ifndef __LASER_DETECTOR_HPP__
#define __LASER_DETECTOR_HPP__

// Orocos
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>

#include "laser_detector/kpa101.h"

// Eigen
// #include <Eigen/Dense>
// RTT-ROS Utilities
// #include <rtt_ros_kdl_tools/tools.hpp>
// #include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/concept_check.hpp>

class read_detector : public RTT::TaskContext{
    public:
        read_detector(const std::string& name);
        virtual ~read_detector(){};
        void updateHook();
        bool configureHook();
    protected:
        // Input ports
	RTT::InputPort<std_msgs::Float64MultiArray> port_ec;
        // Output ports
	RTT::OutputPort<std_msgs::Float64MultiArray> port_laser_info;
	RTT::OutputPort<std_msgs::Float64> port_time,port_p_ec,port_ec_lim,port_r_ec;
	std_msgs::Float64MultiArray laser_info_msg,ec_msg;
	Eigen::Matrix<double,3,1> laser_info;
	Eigen::Matrix<double,4,1> laser_info_filtered;
	
	bool bouboule;
	int cnt;
	std_msgs::Float64 cnt_msgs,ec_lim,p_ec,r_ec;
};

#endif