#pragma once

#include "HealerTypes.h"
#include <string>

namespace SPMeshHealer {

bool readGoZ(const std::string& path, RawMeshData& out);
bool writeGoZ(const std::string& path, const RawMeshData& data);

} // namespace SPMeshHealer
