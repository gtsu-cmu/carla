#ifndef MAIN_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define MAIN_H

#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

#include <carla/client/detail/Client.h>
#include <carla/client/Vehicle.h>
#include <carla/geom/Vector3D.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <carla/rpc/OpendriveGenerationParameters.h>


#include <boost/date_time/posix_time/posix_time.hpp>
#include <common/recgeometry/recGeometry.h>
#include <interfaces/CarlaBridgeInterface/Input/Abstract.h>
#include <interfaces/CarlaBridgeInterface/Output/Abstract.h>
#include <interfaces/VehicleState/Input/Abstract.h>
#include <interfaces/DriveCommand/Output/Abstract.h>
#include <interfaces/VehicleState/Output/Abstract.h>
#include <interfaces/DriveCommand/Input/Abstract.h>

#undef snprintf
#undef carla_client_cpp

class carla_client_cpp{
	public:
		carla_client_cpp(){} //Default contructor
		//static carla::client::Client client1;
		//static std::vector<std::__cxx11::basic_string<char> > temp;
		carla::geom::Transform transform;
		carla::SharedPtr<carla::client::Map> map;
		//carla::client::Client client1;
        carla::SharedPtr<carla::client::Actor> spectator;

		carla::client::BlueprintLibrary::const_pointer camera_bp;
		carla::SharedPtr<carla::client::Actor> cam_actor;
        carla::geom::Transform camera_transform;
		//void *map;
      	boost::shared_ptr<carla::client::Sensor> camera;
		carla::SharedPtr<carla::client::BlueprintLibrary>  blueprint_library;
		//void *blueprint;
		char town_name[100];
        carla::client::World world;
		//void *world;
		carla::client::ActorBlueprint blueprint;
		carla::SharedPtr<carla::client::BlueprintLibrary> vehicles;
		boost::shared_ptr<carla::client::Vehicle> vehicle;
		carla::SharedPtr<carla::client::Actor> actor;
		//boost::static_pointer_cast<cc::Vehicle> vehicle;
		int game_loop_cpp_initialize(int argc, const char *argv[]);
		int game_loop_cpp_executive(DriveCommand command, VehicleState *vehicleState);
		int game_loop_cpp_cleanup(int argc, const char *argv[]);
};

//int game_loop_cpp_initialize(int argc, const char *argv[]);
//int game_loop_cpp_executive(int argc, const char *argv[]);
//int game_loop_cpp_cleanup(int argc, const char *argv[]);
/*
{
	int a=5+argc+sizeof(argv[0]);
	return a;
}
*/
#endif
