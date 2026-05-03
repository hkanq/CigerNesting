#include "import/importer.h"

#include "import/dxf_parser.h"
#include "import/plt_parser.h"
#include "import/svg_parser.h"
#include <algorithm>
#include <cwctype>
#include <filesystem>

namespace nest {

ImportResult Importer::importFile(const std::wstring& path, double flattenTolerance) const {
    std::wstring ext = std::filesystem::path(path).extension().wstring();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](wchar_t ch) { return static_cast<wchar_t>(std::towlower(ch)); });

    if (ext == L".svg") {
        return SvgParser{}.parseFile(path, flattenTolerance);
    }
    if (ext == L".plt" || ext == L".hpgl") {
        return PltParser{}.parseFile(path);
    }
    if (ext == L".dxf") {
        return DxfParser{}.parseFile(path, flattenTolerance);
    }

    return {false, L"Unsupported file format", {}};
}

} // namespace nest
