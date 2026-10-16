// UDP pose plugin for Gazebo 11 (classic).
// Receives packets of 6 float32 (x y z roll pitch yaw) on a UDP port and applies the
// newest pose to the (kinematic) model every simulation step -> smooth motion.
// SDF:  <plugin name="udp_pose" filename="libudp_pose_plugin.so"><port>9090</port></plugin>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace gazebo
{
class UdpPosePlugin : public ModelPlugin
{
public:
  ~UdpPosePlugin() override
  {
    if (sock_ >= 0) close(sock_);
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    int port = sdf->HasElement("port") ? sdf->Get<int>("port") : 0;
    if (port <= 0)
    {
      gzerr << "[udp_pose] <port> missing for model " << model->GetName() << "\n";
      return;
    }
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    int reuse = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
      gzerr << "[udp_pose] cannot bind UDP port " << port << "\n";
      close(sock_);
      sock_ = -1;
      return;
    }
    fcntl(sock_, F_SETFL, O_NONBLOCK);
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&UdpPosePlugin::OnUpdate, this));
    gzmsg << "[udp_pose] model " << model->GetName() << " listening on UDP " << port << "\n";
  }

private:
  void OnUpdate()
  {
    float v[6];
    bool got = false;
    float buf[16];
    ssize_t n;
    while ((n = recv(sock_, buf, sizeof(buf), 0)) == static_cast<ssize_t>(sizeof(v)))
    {
      std::copy(buf, buf + 6, v);   // keep only the newest packet
      got = true;
    }
    if (got)
      model_->SetWorldPose(ignition::math::Pose3d(v[0], v[1], v[2], v[3], v[4], v[5]));
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  int sock_ = -1;
};
GZ_REGISTER_MODEL_PLUGIN(UdpPosePlugin)
}  // namespace gazebo
