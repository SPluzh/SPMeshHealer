#pragma once

#include "HealerTypes.h"
#include <string>

namespace SPMeshHealer {

bool readBinary(const std::string& path, RawMeshData& out);
bool writeBinary(const std::string& path, const RawMeshData& data);

} // namespace SPMeshHealer
