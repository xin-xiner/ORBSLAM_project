#include <stdlib.h>
#include <stdio.h>

#include "FrameReader.h"
#include <iostream>
#include <string>
#include <thread>
#include <mutex>





FrameReader* reader_pointer;


int main(int argc, char** argv)
{
	
	//************************************


	if (argc < 4)
	{
		std::cout << "need more parameters" << std::endl;
		exit(-1);
	}
	

	std::string image_path = argv[1];
	std::string trajectory_path = argv[2];
	std::string camera_info_path = argv[3];



	FrameReader reader(image_path + "\\depth\\", image_path + "\\rgb\\", trajectory_path, camera_info_path);
	std::cout << argc << " " << argv[4] << std::endl;
	if (argc == 5 && std::string(argv[4]) == "trajectory")
	{
		reader.setOnlyTrajectory();
		std::cout << "only trajectory"<<std::endl;
	}
	reader_pointer = &reader;


	std::thread generate_model_thread(&FrameReader::generate_model, &reader);
	


	

	generate_model_thread.join();
	
	return 0;
}