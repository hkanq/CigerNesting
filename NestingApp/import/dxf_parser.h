#pragma once

#include "import/import_result.h"
#include <string>

namespace nest {

class DxfParser {
public:
    ImportResult parseFile(const std::wstring& path, double flattenTolerance) const;
    ImportResult parseText(const std::string& text, double flattenTolerance) const;
};

} // namespace nest
