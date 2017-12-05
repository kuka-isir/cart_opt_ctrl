#ifndef KPA101_H
#define KPA101_H

#include <string>

class KPA101
{

public:
    KPA101();

	bool open_device(std::string serial="69250416");

    bool init_communication();
    
	bool close_communication();
	
	bool init_device();
	
	bool print_device_infos();
	
	// timeout in s
	bool get_data(unsigned int timeout, float* xpos, float* ypos, float* sum);
    
private:
    int m_deviceHandle;
};

#endif // KPA101_H
