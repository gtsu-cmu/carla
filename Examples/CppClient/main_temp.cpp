#include "main.h"
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <cmath>
#include <cstdlib>
#include <stdlib.h>

#include <carla/client/detail/Client.h>
#include <carla/client/Client.h>
#include <carla/client/Vehicle.h>
#include <carla/geom/Vector3D.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/road/Map.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <carla/rpc/OpendriveGenerationParameters.h>
#include <carla/sensor/data/GnssMeasurement.h>

#undef snprintf

#define WHEELBASE 2.946
#define MAXCURV 1.073 //(0.19)//(0.17696305) 

/*
class carla_client_cpp{
  public:
    //static carla::client::Client client1;
    //static std::vector<std::__cxx11::basic_string<char> > temp;
    static boost::shared_ptr<carla::client::Vehicle> vehicle;
    static carla::SharedPtr<carla::client::Actor> actor;
    //boost::static_pointer_cast<cc::Vehicle> vehicle;
    int game_loop_cpp_initialize(int argc, const char *argv[]);
    int game_loop_cpp_executive(int argc, const char *argv[]);
    int game_loop_cpp_cleanup(int argc, const char *argv[]);
};
*/

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
  std::cout<<"Saving image to disk"<<'\n';
  auto filename = "_images/"s + buffer + ".png";
  //auto filename = "/home/shounak/shounak/carla/Examples/CppClient/_images/"s + buffer + ".png";
  // auto view = ImageView::MakeColorConvertedView(
  //     ImageView::MakeView(image),
  //     ColorConverter::CityScapesPalette());
  auto view = ImageView::MakeView(image);
  ImageIO::WriteView(filename, view);
}

static auto ParseArguments(int argc, const char *argv[]) {
  EXPECT_TRUE((argc == 1u) || (argc == 3u));
  using ResultType = std::tuple<std::string, uint16_t>;
  return argc == 3u ?
      ResultType{argv[1u], std::stoi(argv[2u])} :
      ResultType{"localhost", 2000u};
} 





//carla::client::Client carla_client_cpp::client1;
//std::vector<std::__cxx11::basic_string<char> > carla_client_cpp::temp;
//boost::shared_ptr<carla::client::Vehicle> carla_client_cpp::vehicle;
//carla::SharedPtr<carla::client::Actor> carla_client_cpp::actor;
//Main fn modified
/*
carla_client_cpp::carla_client_cpp(){
 
}
*/
int carla_client_cpp::game_loop_cpp_initialize(int argc, const char *argv[]) {
    try {

      std::string host;
      uint16_t port;
      std::tie(host, port) = ParseArguments(argc, argv);
 
      std::mt19937_64 rng((std::random_device())());
      //std::mt19937_64 rng((4)());//std::random_device())());

      //auto client = cc::Client(host, port);
      auto client1 = cc::Client(host, port);
      /*
      temp=client1.GetAvailableMaps();
      std::cout<<"IN GAME LOOP"<<'\n';
      for(unsigned int i=0; i < temp.size(); i++)
        std::cout << temp.at(i) << ' ';
        */
      //printf("maps are:%s\n",client.GetAvailableMaps());
      client1.SetTimeout(300s);
      std::cout << "Client API version3 : " << client1.GetClientVersion() << '\n';
      std::cout << "Server API version : " << client1.GetServerVersion() << '\n';
      
      std::cout<< "After printing versions!" << '\n';
      

      // Load a random town.
      //std::cout << "maps are:" << client.GetAvailableMaps()<<'\n';
      //strcpy(town_name,"/Game/Carla/Maps/Town01");//RandomChoice(client.GetAvailableMaps(), rng);
      //strcpy(town_name,"/Game/Carla/Maps/Town01");
      std::cout << "Loading world: " << town_name << std::endl;
      //TODO uncomment world=client1.LoadWorld(town_name);
      //strcpy(town_name,"/Game/Carla/Maps/bakery_square");
      // const char *town_name_xodr2=std::getenv("PATH");
      // if(town_name_xodr2==NULL)
      //   std::cout<<"WRONG2";
      // std::string str2(town_name_xodr2);
      // std::cout<<"path: "<<town_name_xodr2<<'\n';
      // std::cout<<"path: "<<str2<<'\n';

      // const char *town_name_xodr3=std::getenv("UC_DIR");
      // if(town_name_xodr3==NULL)
      //   std::cout<<"WRONG3";
      // std::string str3(town_name_xodr3);
      // std::cout<<"ucdir file path: "<<town_name_xodr3<<'\n';

      // const char *town_name_xodr=std::getenv("XODR_PATH");
      // if(town_name_xodr==NULL)
      //   std::cout<<"WRONG";
      // std::string str(town_name_xodr);
      // std::cout<<"xodr file path: "<<town_name_xodr<<'\n';

      // std::string town_name_xodr;// ("/home/shounak/shounak/osm2xodr/kansas.xodr");
      // strcpy(town_name_xodr,env_p);
      std::string town_name_xodr ("/home/gregory/workspace/scenarios/kansas.xodr");
      //std::string town_name_xodr ("/home/gregory/workspace/carla/PythonAPI/util/maps/kansas.xodr");
      //std::string town_name_xodr ("kansas.xodr");
      const carla::rpc::OpendriveGenerationParameters params = {2.0,50.0,0.2,0.6,true,true,true};

      /*
  double vertex_distance = 2.0;
    double max_road_length = 50.0;
    double wall_height = 1.0;
    double additional_width = 0.6;
    bool smooth_junctions = true;
    bool enable_mesh_visibility = true;
    bool enable_pedestrian_navigation = true;
      */
      std::cout<<"Town name xodr="<<town_name_xodr;
      world=client1.GenerateOpenDriveWorld(town_name_xodr,params);
      //std::cout << typeid(world).name() << std::endl;

      // Get a random vehicle blueprint.
      blueprint_library = world.GetBlueprintLibrary();
      vehicles = blueprint_library->Filter("vehicle");
      //blueprint = (carla::client::ActorBlueprint*)&RandomChoice(*vehicles, rng);
      blueprint = RandomChoice(*vehicles, rng);

//      carla::client::ActorBlueprint blueprint2= RandomChoice(*vehicles, rng);
      //carla::SharedPtr<carla::client::Map> *map3;
//      map3= static_cast<carla::SharedPtr<carla::client::Map>*> (map);

//      transform = RandomChoice((*map2).GetRecommendedSpawnPoints(), rng);


      // Randomize the blueprint.
      /*
      if ((blueprint).ContainsAttribute("color")) {
        auto &attribute = (blueprint).GetAttribute("color");
        (blueprint).SetAttribute(
            "color",
            RandomChoice(attribute.GetRecommendedValues(), rng));
      }
      */
      // Find a valid spawn point.
      //carla::SharedPtr<carla::client::Map> map2= world.GetMap();
      //carla::SharedPtr<carla::client::Map> *map3;
      //map = &map2;
//      map3= static_cast<carla::SharedPtr<carla::client::Map>*> (map);
      map=world.GetMap();
//      transform = RandomChoice((*map2).GetRecommendedSpawnPoints(), rng);
      //transform = RandomChoice((map)->GetRecommendedSpawnPoints(), rng);
      std::cout<<"SPAWN LOC init:"<<transform.location.x<<" "<<transform.location.y<<" "<<transform.location.z<<'\n';
      
      auto spawn_points=(map)->GetRecommendedSpawnPoints();

      for (auto element: spawn_points){
          std::cout<<"SPAWN LOC:"<<element.location.x<<" "<<element.location.y<<" "<<element.location.z<<'\n';
      }

      //Based on converted value, find closest point in list (Manhattan of x and y)
      //For x, just use the associated alt
      //Input values from Python converter here
      transform.location.x=590.3;
      transform.location.y=-1930.5050425113;
      transform.location.z=0;
      transform.rotation.pitch=0;
      transform.rotation.roll=0;
      transform.rotation.yaw=0;

      float min=INT_MAX;
      auto temp=transform;
      for (auto element: spawn_points){
          float temp_dist=abs(transform.location.x-element.location.x)+abs(transform.location.y-element.location.y);
          if(temp_dist<=min){
            min=temp_dist;
            temp=element;  
          }
      }
      std::cout<<"MIN DISTANCE=  "<<min<<'\n';
      transform=temp;

      // transform.location.x=92.1099 ;
      // transform.location.y=170.544 ;
      // transform.location.z=0.3;
      // transform.rotation.pitch=0;
      // transform.rotation.roll=0;
      // transform.rotation.yaw=-90.003;

      // std::cout<<"x,y,z coordinates are:"<<transform.location.x<<" "<<transform.location.y<<" "<<transform.location.z<<" "<<'\n';
      // std::cout<<"Rotation variables are:"<<transform.rotation.pitch<<" "<<transform.rotation.roll<<" "<<transform.rotation.yaw<<" "<<'\n';
      // // Spawn the vehicle.
      // //char abc=blueprint;
      // auto waypoint_start=map->GetWaypoint((transform.location),true,2);//TODO 2 should e replaced by driving state from Lane.h by including headerfile
      // auto lane_start=waypoint_start->GetLaneId();

      // auto road_start=waypoint_start->GetRoadId();
      // std::cout<<"start road and lane IDs are: "<<road_start<<" "<<lane_start<<'\n';

      //auto roadid_start=carla::client::Waypoint::GetRoadId(waypoint_start);
      //auto road_name=road_start.GetName();
      //std::cout<<"road name from Waypoint="<<road_name;
      //auto laneid_start=carla::client::Waypoint::GetLaneId(waypoint_start);
      //std::cout<<"lane id from Waypoint="<<laneid_start;

      actor = world.SpawnActor(blueprint, transform);
      std::cout << "Spawned " << actor->GetDisplayId() << '\n';
      //auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

      
      vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);
      if(vehicle){
        std::cout<<"Vehicle still detected"<<'\n';
      }
      if(!vehicle){
        std::cout<<"Vehicle NULL"<<'\n';
      }
      //printf("");
      // Apply control to vehicle.
      //cc::Vehicle::Control control;
      //control.throttle = 1.0f;
      //vehicle->ApplyControl(control);
      
      // Move spectator so we can see the vehicle from the simulator window.
      spectator = world.GetSpectator();
      transform.location += 32.0f * transform.GetForwardVector();
      transform.location.z += 2.0f;
      transform.rotation.yaw += 180.0f;
      transform.rotation.pitch = -15.0f;
      spectator->SetTransform(transform);
 
      // Find a camera blueprint.
      //camera_bp = blueprint_library->Find("sensor.camera.semantic_segmentation");
      camera_bp = blueprint_library->Find("sensor.camera.rgb");
      EXPECT_TRUE(camera_bp != nullptr);

      //Spawn a camera attached to the vehicle.
       camera_transform = cg::Transform{
          cg::Location{-5.5f, 0.0f, 2.8f},   // x, y, z.
          cg::Rotation{-15.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.

      //camera_transform=transform;      

      cam_actor = world.SpawnActor(*camera_bp, camera_transform, actor.get());
      camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);

      // Register a callback to save images to disk.
      
      camera->Listen([](auto data) {
        auto image = boost::static_pointer_cast<csd::Image>(data);
        EXPECT_TRUE(image != nullptr);
        SaveSemSegImageToDisk(*image);
      });
      
      std::this_thread::sleep_for(10s);
      
      // Remove actors from the simulation.
      //camera->Destroy();
      //vehicle->Destroy();
      //std::cout << "Actors destroyed." << std::endl;

    } catch (const cc::TimeoutException &e) {
      std::cout << '\n' << e.what() << std::endl;
      return 1;
    } catch (const std::exception &e) {
      std::cout << "\nException: " << e.what() << std::endl;
      return 2;
    }
    return 3;
  }

int carla_client_cpp::game_loop_cpp_executive(DriveCommand command, VehicleState *vehicleState) {
      std::cout<<"start carla exec"<<'\n';
      //carla::sensor::data::GnssMeasurement gnss;
      //std::cout<<"long="<<gnss.GetLongitude()<<'\n';
      //std::string host;
      //uint16_t port;
//      std::tie(host, port) = ParseArguments(argc, argv);
      std::cout<<"IN CARLA EXEC"<<command.desiredCurvature_k<<std::endl;
      
      // Apply control to vehicle.
      if(vehicle){
        std::cout<<"Vehicle still detected 2"<<'\n';
      }
      if(!vehicle){
        std::cout<<"Vehicle NULL"<<'\n';
      }
      cc::Vehicle::Control control_exec;
      control_exec.throttle = 0.0f;
      float strcmd= atan( command.desiredCurvature_k *WHEELBASE)/1.5708; //Normalized by pi/2, more exact should be 1.2645 from max desiredcurvature
      //Need to find range of desiredCurvature so that normalized steer value can be calculated
      std::cout<<"STRCMD="<<strcmd;
      if(strcmd>1){
        control_exec.steer=1;
      }
      if(strcmd<-1){
        control_exec.steer=-1;
      }
      else{
        control_exec.steer=strcmd;
      }
  
      vehicle->ApplyControl(control_exec);
      //carla::geom::Vector3D velocity = {1.0f, 0.0f, 0.0f};
      carla::geom::Vector3D velocity = {command.desiredSpeed_mps, 0.0f, 0.0f};
      vehicle->SetVelocity_SS(velocity);

      RecPose3D zero(0.0,0.0,0.0,0.0,0.0,0.0);
      RecDifferentialPose3D zero_diff(0.0,0.0,0.0,0.0,0.0,0.0);
      vehicleState->acceleration = RecPoint3D(0,0,0);
      vehicleState->poseSTD = zero;    ///< error in position estimate
      vehicleState->velocitySTD = zero_diff; ///< error in the velocity estimate
      vehicleState->curvatureSTD_k = 0.0;
      vehicleState->utmPlane=46;//egoPositionUTM.zone;
      boost::posix_time::ptime time = boost::posix_time::microsec_clock::universal_time();
      vehicleState->timeStamp = time;

      //vehicleState.pose = RecPose3D(position[0], position[1], position[2],orientation[2], orientation[1], orientation[0]);

      //Euler angles is ZYX if pitch,roll,yaw is XYZ

      carla::geom::BoundingBox bounding_box;
      bounding_box=vehicle->GetBoundingBox();
      std::cout<<"bounding_box valuesdist= "<<bounding_box.location.x<<" "<<bounding_box.location.y<<" "<<bounding_box.location.z<<'\n';
      std::cout<<"bounding_box values= "<<bounding_box.rotation.yaw<<" "<<bounding_box.rotation.pitch<<" "<<bounding_box.rotation.roll<<'\n';
      //Can convert x,y,z to CADRE compatible ones after sync
      vehicleState->pose = RecPose3D(bounding_box.location.x,bounding_box.location.y,bounding_box.location.z,bounding_box.rotation.yaw,bounding_box.rotation.pitch,bounding_box.rotation.roll);//RecPose3D(egoPositionUTM.northing, egoPositionUTM.easting, 0, (math::Angle::HalfPi.Radian() - egoOrientation.getYaw().valueRadians()), 0, 0);

      //TVector3D velocity(0) ;
      //TVector3D angularvelocity(0);

      //vehicleState.velocity = RecDifferentialPose3D(velocity.x, velocity.z, 0, 0, 0, 0) ;

      vehicleState->curvature_k = command.desiredCurvature_k;//tan((cars[0]->getCarDynamics()->getWheelSteerAngle(WheelPosition::FRONT_LEFT))/57.2958)/2.8194;    ///< Curvature of the vehicle - as estimated by the pose system
      vehicleState->speed_mps = command.desiredSpeed_mps;//cars[0]->getSpeed(); //math_tools::Length(velocity);
      


      // vehicle-> set wheel angle to 
       //vehicle->ApplyControl(control_exec);
      
      /*
      cc::Vehicle::Control control;
      control.throttle = 1.0f;
      std::cout <<"before segfault"<<'\n';
      vehicle->ApplyControl(control);
      */
            //vehicle ++;
      //printf("vehnum=%d",vehicle);
      std::cout<<"end carla exec"<<'\n';
  return 0;
}

int carla_client_cpp::game_loop_cpp_cleanup(int argc, const char *argv[]) {
      std::string host;
      uint16_t port;
      std::tie(host, port) = ParseArguments(argc, argv);
      //vehicle->Destroy();
    return 0;
}

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
