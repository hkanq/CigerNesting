#include "import/plt_parser.h"

#include "core/units.h"
#include "geometry/winding.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

namespace nest {
namespace {

std::string readTextFile(const std::wstring& path) {
    std::ifstream file(std::filesystem::path(path), std::ios::binary);
    if (!file) {
        return {};
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::string trimUpper(std::string value) {
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) { return !std::isspace(ch); }));
    value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), value.end());
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
    return value;
}

void skipSeparators(const std::string& text, size_t& pos) {
    while (pos < text.size() && (std::isspace(static_cast<unsigned char>(text[pos])) || text[pos] == ',')) {
        ++pos;
    }
}

bool readNumber(const std::string& text, size_t& pos, double& value) {
    skipSeparators(text, pos);
    if (pos >= text.size()) {
        return false;
    }
    const char* begin = text.c_str() + pos;
    char* end = nullptr;
    value = std::strtod(begin, &end);
    if (end == begin) {
        return false;
    }
    pos = static_cast<size_t>(end - text.c_str());
    return true;
}

void finishPolyline(std::vector<Vec2>& polyline, std::vector<Part>& parts, size_t& counter) {
    if (polyline.size() < 2) {
        polyline.clear();
        return;
    }

    Ring ring;
    ring.points = polyline;
    if (ring.points.size() > 2 && !almostEqual(ring.points.front(), ring.points.back())) {
        ring.points.push_back(ring.points.front());
    }
    ring.winding = windingDirection(ring.points);

    Part part;
    part.name = L"hpgl-part-" + std::to_wstring(counter++);
    part.rings.push_back(std::move(ring));
    part.updateDerivedGeometry();
    parts.push_back(std::move(part));
    polyline.clear();
}

} // namespace

ImportResult PltParser::parseFile(const std::wstring& path) const {
    const auto text = readTextFile(path);
    if (text.empty()) {
        return {false, L"PLT/HPGL file could not be read", {}};
    }
    return parseText(text);
}

ImportResult PltParser::parseText(const std::string& text) const {
    ImportResult result;
    result.ok = true;

    bool absoluteMode = true;
    bool penDown = false;
    Vec2 current{};
    std::vector<Vec2> polyline;
    size_t counter = 1;

    size_t begin = 0;
    while (begin < text.size()) {
        const size_t end = text.find(';', begin);
        const std::string raw = text.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
        begin = end == std::string::npos ? text.size() : end + 1;

        const std::string statement = trimUpper(raw);
        if (statement.size() < 2) {
            continue;
        }
        const std::string cmd = statement.substr(0, 2);
        const std::string args = statement.size() > 2 ? statement.substr(2) : std::string{};

        if (cmd == "IN") {
            finishPolyline(polyline, result.parts, counter);
            current = {};
            absoluteMode = true;
            penDown = false;
        } else if (cmd == "PA") {
            absoluteMode = true;
        } else if (cmd == "PR") {
            absoluteMode = false;
        } else if (cmd == "PU" || cmd == "PD") {
            const bool targetPenDown = cmd == "PD";
            if (!targetPenDown && penDown) {
                finishPolyline(polyline, result.parts, counter);
            }
            penDown = targetPenDown;
            if (penDown && polyline.empty()) {
                polyline.push_back(current);
            }

            size_t pos = 0;
            while (pos < args.size()) {
                double x = 0.0;
                double y = 0.0;
                if (!readNumber(args, pos, x) || !readNumber(args, pos, y)) {
                    break;
                }
                Vec2 target{hpglToMillimeters(x), hpglToMillimeters(y)};
                if (!absoluteMode) {
                    target += current;
                }
                current = target;
                if (penDown) {
                    polyline.push_back(current);
                }
            }
        } else if (cmd == "SP") {
            continue;
        }
    }

    finishPolyline(polyline, result.parts, counter);
    if (result.parts.empty()) {
        result.ok = false;
        result.message = L"No supported PLT/HPGL geometry was found";
    } else {
        result.message = L"PLT/HPGL imported";
    }
    return result;
}

} // namespace nest
