#pragma once

#include "HealerTypes.h"
#include <string>

namespace SPMeshHealer {

bool readSTL(const std::string& path, RawMeshData& out);
bool writeSTL(const std::string& path, const RawMeshData& data);

} // namespace SPMeshHealer
