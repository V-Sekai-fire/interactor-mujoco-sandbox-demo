#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

namespace human {

std::vector<std::string> files();

mjModel *build(int withhold, std::string &error, std::vector<std::string> &kept);

} // namespace human
