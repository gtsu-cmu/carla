#include<iostream>
#include "main.h"
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include "main.h"

/*
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
*/
int main(){
	//carla::client::Vehicle vehicle;
	//carla::client::Sensor camera;
	//std::tie(vehicle,camera)=game_loop_cpp_initialize(1,NULL);
	carla_client_cpp carla_obj;
	carla_obj.game_loop_cpp_initialize(1,NULL);
	if(!carla_obj.vehicle){
		std::cout<<"Vehicle is NULL\n";
	}
	carla_obj.game_loop_cpp_executive(1,NULL);
	carla_obj.game_loop_cpp_cleanup(1,NULL);
	//game_loop_cpp_executive(vehicle);
	//game_loop_cpp_cleanup(vehicle,camera);
	printf("GOOD WORK! The driver can compile and link main now\n");
	return 0;
}
