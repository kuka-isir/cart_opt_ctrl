#include <iostream>
#include <unistd.h>
#include "cart_opt_ctrl/read_detector.hpp"


read_detector::read_detector(const std::string& name):
RTT::TaskContext(name)
{
    // Here you can add your ports, properties and operations
    // ex : this->addOperation("my_super_function",&ControllerTest::MyFunction,this,RTT::OwnThread);
  this->addPort("LaserInfo",port_laser_info);
  this->addPort("Time",port_time);
  this->addPort("Ec",port_ec);
  this->addPort("Ec_lim",port_ec_lim);
  this->addPort("p_ec",port_p_ec);
  this->addPort("r_ec",port_r_ec);
  this->addProperty("bouboule",bouboule);
  
}

bool read_detector::configureHook()
{
  laser_info.setZero();
  bouboule = false;
  cnt = 0;
  laser_info_filtered.setZero();
}

void read_detector::updateHook()
{
	KPA101 cube;

	if(!cube.open_device())
	{
		cube.close_communication();
// 		return -1;
	}

	if(!cube.init_communication())
	{
		cube.close_communication();
// 		return -1;
	}

	float xpos, ypos, sum;
	cube.get_data(5, &xpos, &ypos, &sum);
	if (sum < 3){
	  ypos = 0;
	  xpos = 0;
	}
	
	laser_info[0] = xpos;
	laser_info[1] = ypos;
	laser_info[2] = sum;
	
	//Filtrage des données 
	
 	for(int i=0; i<3 ; i++){
 	  laser_info_filtered(i) = 0.2 * laser_info_filtered(i) + 0.8 * laser_info(i);
 	}
 	
 	laser_info_filtered(3) = std::sqrt(std::pow(laser_info_filtered(0),2)+std::pow(laser_info_filtered(1),2));
 	
// 	
	tf::matrixEigenToMsg(laser_info_filtered,laser_info_msg);
	
	
	port_laser_info.write(laser_info_msg);
	cube.close_communication();

  
}


// Let orocos know how to create the component
ORO_CREATE_COMPONENT(read_detector)
