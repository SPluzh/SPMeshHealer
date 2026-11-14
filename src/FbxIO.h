#pragma once

#include "HealerTypes.h"
#include <string>

namespace SPMeshHealer {

bool readFBX(const std::string& path, RawMeshData& out);
bool writeFBX(const std::string& path, const RawMeshData& data);

} // namespace SPMeshHealer
