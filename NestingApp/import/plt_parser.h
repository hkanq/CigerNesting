#pragma once

#include "import/import_result.h"
#include <string>

namespace nest {

class PltParser {
public:
    ImportResult parseFile(const std::wstring& path) const;
    ImportResult parseText(const std::string& text) const;
};

} // namespace nest
