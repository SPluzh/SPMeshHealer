#pragma once

#include "HealerTypes.h"
#include <string>

namespace SPMeshHealer {

bool readOBJ(const std::string& path, RawMeshData& out);
bool writeOBJ(const std::string& path, const RawMeshData& data);

} // namespace SPMeshHealer
