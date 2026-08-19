#include <chrono>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <gz/msgs/twist.pb.h>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
class BoatWaveFollower
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/) override
  {
    this->entity = _entity;
    this->canonicalLink = Model(_entity).CanonicalLink(_ecm);

    if (auto poseComp = _ecm.Component<components::Pose>(this->entity))
    {
      this->basePose = poseComp->Data();
    }
    else
    {
      _ecm.CreateComponent(this->entity, components::Pose(this->basePose));
    }

    this->meanZ = this->basePose.Pos().Z();

    if (_sdf->HasElement("mean_z"))
      this->meanZ = _sdf->Get<double>("mean_z");
    if (_sdf->HasElement("heave_amplitude"))
      this->heaveAmplitude = _sdf->Get<double>("heave_amplitude");
    if (_sdf->HasElement("roll_amplitude"))
      this->rollAmplitude = _sdf->Get<double>("roll_amplitude");
    if (_sdf->HasElement("pitch_amplitude"))
      this->pitchAmplitude = _sdf->Get<double>("pitch_amplitude");
    if (_sdf->HasElement("primary_frequency"))
      this->primaryFrequency = _sdf->Get<double>("primary_frequency");
    if (_sdf->HasElement("secondary_frequency"))
      this->secondaryFrequency = _sdf->Get<double>("secondary_frequency");
    if (_sdf->HasElement("wave_number_x"))
      this->waveNumberX = _sdf->Get<double>("wave_number_x");
    if (_sdf->HasElement("wave_number_y"))
      this->waveNumberY = _sdf->Get<double>("wave_number_y");
    if (_sdf->HasElement("velocity_topic"))
      this->velocityTopic = _sdf->Get<std::string>("velocity_topic");

    if (!this->velocityTopic.empty())
    {
      if (this->velocityTopic.front() != '/')
      {
        if (auto name = _ecm.Component<components::Name>(this->entity))
        {
          this->velocityTopic =
              "/model/" + name->Data() + "/" + this->velocityTopic;
        }
      }
      this->node.Subscribe(
          this->velocityTopic, &BoatWaveFollower::OnVelocity, this);
    }
  }

  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    if (this->canonicalLink == kNullEntity)
      return;

    // Physics-owned link velocities preserve contact response. The previous
    // WorldPoseCmd integration teleported through terrain and other hulls.
    Link link(this->canonicalLink);
    link.SetLinearVelocity(
        _ecm, math::Vector3d(this->linearVelocity.load(), 0.0, 0.0));
    link.SetAngularVelocity(
        _ecm, math::Vector3d(0.0, 0.0, this->angularVelocity.load()));
  }

  private: void OnVelocity(const gz::msgs::Twist &_msg)
  {
    this->linearVelocity.store(_msg.linear().x());
    this->angularVelocity.store(_msg.angular().z());
  }

  private: Entity entity{kNullEntity};
  private: Entity canonicalLink{kNullEntity};
  private: math::Pose3d basePose{0, 0, 0, 0, 0, 0};
  private: double meanZ{0.42};
  private: double heaveAmplitude{0.1};
  private: double rollAmplitude{0.055};
  private: double pitchAmplitude{0.045};
  private: double primaryFrequency{1.1};
  private: double secondaryFrequency{1.7};
  private: double waveNumberX{0.35};
  private: double waveNumberY{0.22};
  private: std::string velocityTopic;
  private: gz::transport::Node node;
  private: std::atomic<double> linearVelocity{0.0};
  private: std::atomic<double> angularVelocity{0.0};
};
}
}
}
}

GZ_ADD_PLUGIN(gz::sim::systems::BoatWaveFollower,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::BoatWaveFollower,
                    "gz::sim::systems::BoatWaveFollower")
