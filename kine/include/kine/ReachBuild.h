// Copyright by BeeX [2026]

#ifndef KINE_REACHBUILD_H
#define KINE_REACHBUILD_H

#include <kine/ArmBody.h>
#include <kine/Reach.h>

namespace kine {

// Solves every cell of the grid described by `spec`, filling in the grid fields as it goes.
// Throws std::invalid_argument when a joint window spans a whole turn, because the table
// could not then mirror the branch solvePosition would pick.
void buildReachTable(const ArmBody &body, const ArmConfig &config, const ReachSpec &spec, ReachTable &out);

}  // namespace kine

#endif  // KINE_REACHBUILD_H
