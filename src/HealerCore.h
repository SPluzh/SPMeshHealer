#pragma once

#include "HealerTypes.h"

namespace SPMeshHealer {

bool healMesh(const RawMeshData& input, RawMeshData& output, const HealerOptions& opts, HealerStats& stats);

} // namespace SPMeshHealer
