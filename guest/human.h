#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

// MS-Human-700 built from the XML tree embedded in this program.
namespace human {

// Paths of the embedded files, relative to the model directory.
std::vector<std::string> files();

// Compiles the model. `withhold` is the index of one embedded file to leave out, or -1; a control
// uses it to show that a missing file stops the build. On failure the reply is nullptr and `error`
// says why. `kept` receives the mesh files the built model still reads.
mjModel *build(int withhold, std::string &error, std::vector<std::string> &kept);

} // namespace human
