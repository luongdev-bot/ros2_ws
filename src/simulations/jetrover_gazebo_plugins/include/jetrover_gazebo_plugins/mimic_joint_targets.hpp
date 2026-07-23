#ifndef JETROVER_GAZEBO_PLUGINS__MIMIC_JOINT_TARGETS_HPP_
#define JETROVER_GAZEBO_PLUGINS__MIMIC_JOINT_TARGETS_HPP_

#include <array>
#include <string>

namespace jetrover::gazebo::systems
{
struct MimicJointTarget
{
  std::string name;
  double multiplier;
  double offset;
};

inline std::array<MimicJointTarget, 5> DefaultMimicJointTargets()
{
  return {{
      {"l_joint", -1.0, 0.0},
      {"l_in_joint", -1.0, 0.0},
      {"l_out_joint", 1.0, 0.0},
      {"r_in_joint", -1.0, 0.0},
      {"r_out_joint", 1.0, 0.0},
    }};
}

inline double ComputeMimicTarget(
  const double drivenPosition,
  const MimicJointTarget &mimic)
{
  return drivenPosition * mimic.multiplier + mimic.offset;
}
}  // namespace jetrover::gazebo::systems

#endif  // JETROVER_GAZEBO_PLUGINS__MIMIC_JOINT_TARGETS_HPP_
