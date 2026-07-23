#include <array>

#include <gtest/gtest.h>

#include "jetrover_gazebo_plugins/mimic_joint_targets.hpp"

namespace
{
using jetrover::gazebo::systems::ComputeMimicTarget;
using jetrover::gazebo::systems::DefaultMimicJointTargets;
using jetrover::gazebo::systems::MimicJointTarget;

TEST(MimicJointTargets, AppliesConfiguredMultipliers)
{
  constexpr double drivenPosition = -1.0;
  const auto joints = DefaultMimicJointTargets();

  EXPECT_DOUBLE_EQ(1.0, ComputeMimicTarget(drivenPosition, joints[0]));
  EXPECT_DOUBLE_EQ(1.0, ComputeMimicTarget(drivenPosition, joints[1]));
  EXPECT_DOUBLE_EQ(-1.0, ComputeMimicTarget(drivenPosition, joints[2]));
  EXPECT_DOUBLE_EQ(1.0, ComputeMimicTarget(drivenPosition, joints[3]));
  EXPECT_DOUBLE_EQ(-1.0, ComputeMimicTarget(drivenPosition, joints[4]));
}

TEST(MimicJointTargets, AppliesOffset)
{
  const MimicJointTarget joint{"jaw", -1.5, 0.25};
  EXPECT_DOUBLE_EQ(1.75, ComputeMimicTarget(-1.0, joint));
}
}  // namespace
