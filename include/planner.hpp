#pragma once

#include <chrono>
#include <optional>

#include "plan_update.pb.h"
#include "world_tick.pb.h"
#include "ego_cmd.pb.h"

namespace navsim {

class Planner {
 public:
  Planner() = default;

  bool solve(const proto::WorldTick& world,
             const std::optional<proto::PlanUpdate>& last_plan,
             std::chrono::milliseconds deadline,
             proto::PlanUpdate* out_plan,
             proto::EgoCmd* out_cmd) const;
};

}  // namespace navsim
