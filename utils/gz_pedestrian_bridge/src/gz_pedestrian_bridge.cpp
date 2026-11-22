// Batched pedestrian pose bridge: ROS arena_people_msgs/srv/UpdatePedestrians ->
// one gz-transport request to /world/<world>/set_pose_vector (gz.msgs.Pose_V).
//
// Replaces N concurrent /world/<world>/set_pose calls through ros_gz_bridge (one per
// pedestrian per tick), whose service queue depth of 10 silently drops replies when more
// than ~10 pedestrians are updated at once.
//
// Uses the non-blocking set_pose_vector (not .../blocking): it only queues the poses in
// UserCommands and replies immediately, so it also answers while the sim is paused.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

#include <arena_people_msgs/srv/update_pedestrians.hpp>

using UpdatePedestrians = arena_people_msgs::srv::UpdatePedestrians;

class GzPedestrianBridge : public rclcpp::Node
{
public:
  GzPedestrianBridge()
  : rclcpp::Node("gz_pedestrian_bridge")
  {
    const auto world = declare_parameter<std::string>("world", "default");
    const auto service = declare_parameter<std::string>(
      "service", "/world/" + world + "/update_pedestrians");
    timeout_ms_ = static_cast<unsigned int>(declare_parameter<int>("timeout_ms", 1000));
    gz_service_ = "/world/" + world + "/set_pose_vector";

    service_ = create_service<UpdatePedestrians>(
      service,
      std::bind(&GzPedestrianBridge::handle, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Serving %s -> gz %s", service.c_str(), gz_service_.c_str());
  }

private:
  void handle(
    const std::shared_ptr<UpdatePedestrians::Request> request,
    std::shared_ptr<UpdatePedestrians::Response> response)
  {
    const auto n = request->pedestrians.size();
    if (n == 0) {
      return;
    }

    gz::msgs::Pose_V poses;
    for (const auto & ped : request->pedestrians) {
      auto * pose = poses.add_pose();
      pose->set_name(ped.name);
      auto * p = pose->mutable_position();
      p->set_x(ped.pose.position.x);
      p->set_y(ped.pose.position.y);
      p->set_z(ped.pose.position.z);
      // Always set all four components: an unset gz.msgs.Quaternion has w = 0.
      auto * q = pose->mutable_orientation();
      q->set_x(ped.pose.orientation.x);
      q->set_y(ped.pose.orientation.y);
      q->set_z(ped.pose.orientation.z);
      q->set_w(ped.pose.orientation.w);
    }

    gz::msgs::Boolean reply;
    bool result = false;
    const auto t0 = std::chrono::steady_clock::now();
    const bool executed = gz_node_.Request(gz_service_, poses, timeout_ms_, reply, result);
    const auto ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    const bool ok = executed && result && reply.data();
    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s failed for %zu pedestrians (executed=%d result=%d data=%d, %.1f ms)",
        gz_service_.c_str(), n, executed, result, reply.data(), ms);
    } else {
      RCLCPP_DEBUG(get_logger(), "set %zu pedestrian poses in %.2f ms", n, ms);
    }

    // set_pose_vector reports one flag for the whole batch (unknown names are logged by gz).
    response->results.assign(
      n, ok ? UpdatePedestrians::Response::SUCCESS : UpdatePedestrians::Response::NOT_FOUND);
  }

  gz::transport::Node gz_node_;
  std::string gz_service_;
  unsigned int timeout_ms_{1000};
  rclcpp::Service<UpdatePedestrians>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzPedestrianBridge>());
  rclcpp::shutdown();
  return 0;
}
