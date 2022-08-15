#define _GLIBCXX_USE_CXX11_ABI 0
#include "main.h"
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
//#include "/home/shounak/shounak/carla/LibCarla/source/carla/client/Client.h"
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

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

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;
template <typename T> std::string type_name();

#define EXPECT_TRUE(pred) if (!(pred)) { printf("HERE\n"); throw std::runtime_error(#pred); }

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}

/// Save a semantic segmentation image to disk converting to CityScapes palette.

static void SaveSemSegImageToDisk(const csd::Image &image) {
  using namespace carla::image;

  char buffer[9u];
  std::snprintf(buffer, sizeof(buffer), "%08zu", image.GetFrame());
  auto filename = "_images/"s + buffer + ".png";

  auto view = ImageView::MakeColorConvertedView(
      ImageView::MakeView(image),
      ColorConverter::CityScapesPalette());
  ImageIO::WriteView(filename, view);
}

static auto ParseArguments(int argc, const char *argv[]) {
  EXPECT_TRUE((argc == 1u) || (argc == 3u));
  using ResultType = std::tuple<std::string, uint16_t>;
  return argc == 3u ?
      ResultType{argv[1u], std::stoi(argv[2u])} :
      ResultType{"localhost", 2000u};
} 

//Main fn modified
int game_loop_cpp_initialize(int argc, const char *argv[]) {
  try {

    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);

    std::mt19937_64 rng((std::random_device())());
    //std::mt19937_64 rng((4)());//std::random_device())());

    auto client = cc::Client(host, port);
    auto temp=client.GetAvailableMaps();
    std::cout<<"IN GAME LOOP"<<'\n';
    for(unsigned int i=0; i < temp.size(); i++)
      std::cout << temp.at(i) << ' ';
    //printf("maps are:%s\n",client.GetAvailableMaps());
    client.SetTimeout(100s);
    std::cout << "Client API version3 : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';

    // Load a random town.
    //std::cout << "maps are:" << client.GetAvailableMaps()<<'\n';
    auto town_name = "/Game/Carla/Maps/Town03";//RandomChoice(client.GetAvailableMaps(), rng);
    std::cout << "Loading world: " << town_name << std::endl;
    auto world = client.LoadWorld(town_name);

    // Get a random vehicle blueprint.
    auto blueprint_library = world.GetBlueprintLibrary();
    auto vehicles = blueprint_library->Filter("vehicle");
    auto blueprint = RandomChoice(*vehicles, rng);

    // Randomize the blueprint.
    if (blueprint.ContainsAttribute("color")) {
      auto &attribute = blueprint.GetAttribute("color");
      blueprint.SetAttribute(
          "color",
          RandomChoice(attribute.GetRecommendedValues(), rng));
    }

    // Find a valid spawn point.
    auto map = world.GetMap();
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);

    // Spawn the vehicle.
    auto actor = world.SpawnActor(blueprint, transform);
    std::cout << "Spawned " << actor->GetDisplayId() << '\n';
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

    // Apply control to vehicle.
    
    cc::Vehicle::Control control;
    control.throttle = 1.0f;
    vehicle->ApplyControl(control);
    
    // Move spectator so we can see the vehicle from the simulator window.
    auto spectator = world.GetSpectator();
    transform.location += 32.0f * transform.GetForwardVector();
    transform.location.z += 2.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -15.0f;
    spectator->SetTransform(transform);

    // Find a camera blueprint.
    auto camera_bp = blueprint_library->Find("sensor.camera.semantic_segmentation");
    EXPECT_TRUE(camera_bp != nullptr);

    // Spawn a camera attached to the vehicle.
    auto camera_transform = cg::Transform{
        cg::Location{-5.5f, 0.0f, 2.8f},   // x, y, z.
        cg::Rotation{-15.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    auto cam_actor = world.SpawnActor(*camera_bp, camera_transform, actor.get());
    auto camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);
    

    std::cout<<"data type of vehicle:"<<type_name<decltype(vehicle)>();
    std::cout<<"data type of vehicle:"<<type_name<decltype(camera)>();
    // Register a callback to save images to disk.
    
    camera->Listen([](auto data) {
      auto image = boost::static_pointer_cast<csd::Image>(data);
      EXPECT_TRUE(image != nullptr);
      SaveSemSegImageToDisk(*image);
    });
    
    std::this_thread::sleep_for(10s);
    //return std::tuple(vehicle,camera);
    // Remove actors from the simulation.
 
  } catch (const cc::TimeoutException &e) {
    std::cout << '\n' << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cout << "\nException: " << e.what() << std::endl;
    return 2;
  }
  return 3;
}
/*
int game_loop_cpp_executive(carla::client::Vehicle vehicle) {
    // Apply control to vehicle.
    cc::Vehicle::Control control;
    control.throttle = 1.0f;
    vehicle->ApplyControl(control);
    return 0;
}

int game_loop_cpp_cleanup(carla::client::Vehicle vehicle,carla::client::Client camera) {
    camera->Destroy();
    vehicle->Destroy();
    std::cout << "Actors destroyed." << std::endl;
    return 0;
}
*/
/*
int main(int argc, const char *argv[]) {
  try {

    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);

    std::mt19937_64 rng((std::random_device())());
    //std::mt19937_64 rng((4)());//std::random_device())());

    auto client = cc::Client(host, port);
    client.SetTimeout(10s);
    std::cout << "Client API version4 : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';

    // Load a random town.
    //std::cout << "maps are:" << client.GetAvailableMaps()<<'\n';
    auto town_name = RandomChoice(client.GetAvailableMaps(), rng);
    std::cout << "Loading world: " << town_name << std::endl;
    auto world = client.LoadWorld(town_name);

    // Get a random vehicle blueprint.
    auto blueprint_library = world.GetBlueprintLibrary();
    auto vehicles = blueprint_library->Filter("vehicle");
    auto blueprint = RandomChoice(*vehicles, rng);

    // Randomize the blueprint.
    if (blueprint.ContainsAttribute("color")) {
      auto &attribute = blueprint.GetAttribute("color");
      blueprint.SetAttribute(
          "color",
          RandomChoice(attribute.GetRecommendedValues(), rng));
    }

    // Find a valid spawn point.
    auto map = world.GetMap();
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);

    // Spawn the vehicle.
    auto actor = world.SpawnActor(blueprint, transform);
    std::cout << "Spawned " << actor->GetDisplayId() << '\n';
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

    // Apply control to vehicle.
    cc::Vehicle::Control control;
    control.throttle = 1.0f;
    vehicle->ApplyControl(control);

    // Move spectator so we can see the vehicle from the simulator window.
    auto spectator = world.GetSpectator();
    transform.location += 32.0f * transform.GetForwardVector();
    transform.location.z += 2.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -15.0f;
    spectator->SetTransform(transform);

    // Find a camera blueprint.
    auto camera_bp = blueprint_library->Find("sensor.camera.semantic_segmentation");
    EXPECT_TRUE(camera_bp != nullptr);

    // Spawn a camera attached to the vehicle.
    auto camera_transform = cg::Transform{
        cg::Location{-5.5f, 0.0f, 2.8f},   // x, y, z.
        cg::Rotation{-15.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    auto cam_actor = world.SpawnActor(*camera_bp, camera_transform, actor.get());
    auto camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);

    // Register a callback to save images to disk.
    camera->Listen([](auto data) {
      auto image = boost::static_pointer_cast<csd::Image>(data);
      EXPECT_TRUE(image != nullptr);
      SaveSemSegImageToDisk(*image);
    });

    std::this_thread::sleep_for(10s);

    // Remove actors from the simulation.
    camera->Destroy();
    vehicle->Destroy();
    std::cout << "Actors destroyed." << std::endl;

  } catch (const cc::TimeoutException &e) {
    std::cout << '\n' << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cout << "\nException: " << e.what() << std::endl;
    return 2;
  }
}
*/
