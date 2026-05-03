#pragma once

#include "import/import_result.h"
#include <string>

namespace nest {

class Importer {
public:
    ImportResult importFile(const std::wstring& path, double flattenTolerance) const;
};

} // namespace nest
