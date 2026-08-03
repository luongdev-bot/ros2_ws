#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/entity.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

#include "jetrover_gazebo_plugins/mimic_joint_targets.hpp"

namespace jetrover::gazebo::systems
{
class GraspVelocitySystem final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    const auto topic = _sdf->Get<std::string>(
      "topic", "/grasp/entity_command").first;
    this->drivenJointName = _sdf->Get<std::string>(
      "driven_joint", "").first;
    if (this->drivenJointName.empty())
    {
      // Accept the more explicit alias as well; both resolve to the same
      // cached entity and retain r_joint as the default.
      this->drivenJointName = _sdf->Get<std::string>(
        "driven_joint_name", "r_joint").first;
    }
    if (this->drivenJointName.empty())
    {
      ignwarn << "Empty driven joint name, using [r_joint]\n";
      this->drivenJointName = "r_joint";
    }
    // r_joint approaches -1.0 when open; release below this threshold.
    this->releaseJointPosition = _sdf->Get<double>(
      "release_joint_position", -0.5).first;

    for (auto &mimic : this->mimicJoints)
    {
      const auto multiplier = _sdf->Get<double>(
        mimic.target.name + "_multiplier", mimic.target.multiplier).first;
      const auto offset = _sdf->Get<double>(
        mimic.target.name + "_offset", mimic.target.offset).first;
      mimic.target.multiplier = std::isfinite(multiplier) ?
        multiplier : mimic.target.multiplier;
      mimic.target.offset = std::isfinite(offset) ?
        offset : mimic.target.offset;
    }

    constexpr double defaultCommandTimeout = 0.5;
    auto commandTimeout = _sdf->Get<double>(
      "command_timeout", defaultCommandTimeout).first;
    using ClockDuration = std::chrono::steady_clock::duration;
    const auto minimumCommandTimeout =
      std::chrono::duration<double>(ClockDuration{1}).count();
    const auto maximumCommandTimeout =
      std::chrono::duration<double>(ClockDuration::max()).count();
    if (!std::isfinite(commandTimeout) ||
      commandTimeout < minimumCommandTimeout ||
      commandTimeout >= maximumCommandTimeout)
    {
      ignwarn << "Invalid grasp command timeout [" << commandTimeout
              << "], using [" << defaultCommandTimeout << "] seconds\n";
      commandTimeout = defaultCommandTimeout;
    }
    this->commandTimeout = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(commandTimeout));
    if (!this->node.Subscribe(topic, &GraspVelocitySystem::OnCommand, this))
    {
      ignerr << "Failed to subscribe to grasp entity commands on ["
             << topic << "]\n";
    }
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    const auto drivenPos = this->UpdateMimicJoints(_ecm);

    std::optional<gz::msgs::Entity> command;
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      command = std::move(this->pendingCommand);
      this->pendingCommand.reset();
    }

    if (this->lastCommandTime.has_value() &&
      _info.simTime < *this->lastCommandTime)
    {
      if (!this->heldName.empty())
      {
        ignwarn << "Releasing grasp on [" << this->heldName
                << "] after simulation time moved backwards\n";
      }
      this->heldName.clear();
      this->heldLinks.clear();
      this->lastCommandTime.reset();
      this->lastType = gz::msgs::Entity::NONE;
      this->lastName.clear();
      return;
    }

    if (command.has_value())
    {
      if (this->ApplyCommand(*command, _ecm))
      {
        this->lastCommandTime = _info.simTime;
      }
    }

    if (drivenPos.has_value() &&
      !this->heldName.empty() &&
      *drivenPos < this->releaseJointPosition)
    {
      this->heldName.clear();
      this->heldLinks.clear();
      this->lastType = gz::msgs::Entity::NONE;
      this->lastName.clear();
    }

    if (!this->heldName.empty())
    {
      if (this->lastCommandTime.has_value() &&
        _info.simTime - *this->lastCommandTime >= this->commandTimeout)
      {
        ignwarn << "Releasing grasp on [" << this->heldName
                << "] after command timeout\n";
        this->heldName.clear();
        this->heldLinks.clear();
        this->lastType = gz::msgs::Entity::NONE;
        this->lastName.clear();
        return;
      }

      const bool hasInvalidLink = std::any_of(
        this->heldLinks.begin(), this->heldLinks.end(),
        [&_ecm](const auto entity) {return !_ecm.HasEntity(entity);});
      if (this->heldLinks.empty() || hasInvalidLink)
      {
        this->heldLinks = this->ModelLinks(this->heldName, _ecm);
      }
      this->ZeroLinks(this->heldLinks, _ecm);
    }
  }

private:
  struct CachedMimicJoint
  {
    MimicJointTarget target;
    gz::sim::Entity entity{gz::sim::kNullEntity};
    std::vector<double> position{0.0};
  };

  static std::array<CachedMimicJoint, 5> DefaultMimicJoints()
  {
    const auto targets = DefaultMimicJointTargets();
    std::array<CachedMimicJoint, 5> joints;
    for (std::size_t index = 0; index < joints.size(); ++index)
    {
      joints[index].target = targets[index];
    }
    return joints;
  }

  static bool ValidEntity(
    const gz::sim::Entity _entity,
    const gz::sim::EntityComponentManager &_ecm)
  {
    return _entity != gz::sim::kNullEntity && _ecm.HasEntity(_entity);
  }

  void ResolveJointEntities(gz::sim::EntityComponentManager &_ecm)
  {
    this->drivenJointEntity = _ecm.EntityByComponents(
      gz::sim::components::Joint(),
      gz::sim::components::Name(this->drivenJointName));
    for (auto &mimic : this->mimicJoints)
    {
      mimic.entity = _ecm.EntityByComponents(
        gz::sim::components::Joint(),
        gz::sim::components::Name(mimic.target.name));
    }

    if (this->ValidEntity(this->drivenJointEntity, _ecm))
    {
      // JointPosition is a state component. Enabling the check once makes it
      // available for reading without adding any command or controller.
      gz::sim::Joint(this->drivenJointEntity).EnablePositionCheck(_ecm);
    }
  }

  bool JointEntitiesNeedRefresh(
    const gz::sim::EntityComponentManager &_ecm) const
  {
    if (!this->ValidEntity(this->drivenJointEntity, _ecm))
    {
      return true;
    }
    return std::any_of(
      this->mimicJoints.begin(), this->mimicJoints.end(),
      [&_ecm, this](const auto &mimic)
      {
        return !this->ValidEntity(mimic.entity, _ecm);
      });
  }

  std::optional<double> UpdateMimicJoints(
    gz::sim::EntityComponentManager &_ecm)
  {
    if (this->JointEntitiesNeedRefresh(_ecm))
    {
      this->ResolveJointEntities(_ecm);
    }

    if (!this->ValidEntity(this->drivenJointEntity, _ecm))
    {
      return std::nullopt;
    }

    const auto position = _ecm.ComponentData<gz::sim::components::JointPosition>(
      this->drivenJointEntity);
    if (!position.has_value() || position->empty())
    {
      return std::nullopt;
    }
    const double drivenPos = position->front();

    for (auto &mimic : this->mimicJoints)
    {
      if (!this->ValidEntity(mimic.entity, _ecm))
      {
        continue;
      }

      const auto target = ComputeMimicTarget(drivenPos, mimic.target);
      mimic.position[0] = target;
      _ecm.SetComponentData<gz::sim::components::JointPositionReset>(
        mimic.entity, mimic.position);
      // SetComponentData does not mark an unchanged value. Marking the reset
      // command explicitly keeps this kinematic update effective every tick,
      // including when the target position is unchanged.
      _ecm.SetChanged(
        mimic.entity,
        gz::sim::components::JointPositionReset::typeId,
        gz::sim::ComponentState::OneTimeChange);
    }
    return drivenPos;
  }

  void OnCommand(const gz::msgs::Entity &_message)
  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->pendingCommand = _message;
  }

  bool ApplyCommand(
    const gz::msgs::Entity &_command,
    gz::sim::EntityComponentManager &_ecm)
  {
    if (_command.type() == gz::msgs::Entity::MODEL &&
      !_command.name().empty())
    {
      if (this->lastType != _command.type() ||
        this->lastName != _command.name())
      {
        this->heldName = _command.name();
        this->heldLinks = this->ModelLinks(this->heldName, _ecm);
      }
    }
    else if (_command.type() == gz::msgs::Entity::NONE &&
      !_command.name().empty())
    {
      if (this->lastType != _command.type() ||
        this->lastName != _command.name())
      {
        if (this->heldName == _command.name())
        {
          this->ZeroLinks(this->heldLinks, _ecm);
          this->heldName.clear();
          this->heldLinks.clear();
        }
        else
        {
          this->ZeroLinks(this->ModelLinks(_command.name(), _ecm), _ecm);
        }
      }
    }
    else
    {
      return false;
    }

    this->lastType = _command.type();
    this->lastName = _command.name();
    return true;
  }

  std::vector<gz::sim::Entity> ModelLinks(
    const std::string &_name,
    const gz::sim::EntityComponentManager &_ecm) const
  {
    const auto entity = _ecm.EntityByComponents(
      gz::sim::components::Model(), gz::sim::components::Name(_name));
    if (entity == gz::sim::kNullEntity)
    {
      return {};
    }
    return gz::sim::Model(entity).Links(_ecm);
  }

  void ZeroLinks(
    const std::vector<gz::sim::Entity> &_links,
    gz::sim::EntityComponentManager &_ecm) const
  {
    for (const auto entity : _links)
    {
      const gz::sim::Link link(entity);
      link.SetLinearVelocity(_ecm, gz::math::Vector3d::Zero);
      link.SetAngularVelocity(_ecm, gz::math::Vector3d::Zero);
    }
  }

  gz::transport::Node node;
  std::mutex commandMutex;
  std::optional<gz::msgs::Entity> pendingCommand;
  std::string heldName;
  std::vector<gz::sim::Entity> heldLinks;
  std::string drivenJointName{"r_joint"};
  gz::sim::Entity drivenJointEntity{gz::sim::kNullEntity};
  double releaseJointPosition{-0.5};
  std::array<CachedMimicJoint, 5> mimicJoints{DefaultMimicJoints()};
  std::chrono::steady_clock::duration commandTimeout{
    std::chrono::milliseconds(500)};
  std::optional<std::chrono::steady_clock::duration> lastCommandTime;
  int lastType{gz::msgs::Entity::NONE};
  std::string lastName;
};
}  // namespace jetrover::gazebo::systems

IGNITION_ADD_PLUGIN(
  jetrover::gazebo::systems::GraspVelocitySystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  jetrover::gazebo::systems::GraspVelocitySystem,
  "jetrover::gazebo::systems::GraspVelocitySystem")
