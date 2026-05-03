#include "import/dxf_parser.h"

#include "core/math_utils.h"
#include "geometry/arc.h"
#include "geometry/winding.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace nest {
namespace {

struct GroupPair {
    int code = 0;
    std::string value;
};

std::string readTextFile(const std::wstring& path) {
    std::ifstream file(std::filesystem::path(path), std::ios::binary);
    if (!file) {
        return {};
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::string trim(const std::string& text) {
    const auto begin = std::find_if_not(text.begin(), text.end(), [](unsigned char ch) { return std::isspace(ch); });
    const auto end = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char ch) { return std::isspace(ch); }).base();
    if (begin >= end) {
        return {};
    }
    return {begin, end};
}

std::string upper(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
    return text;
}

double toDouble(const std::string& value, double fallback = 0.0) {
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    return end == value.c_str() ? fallback : parsed;
}

int toInt(const std::string& value, int fallback = 0) {
    char* end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    return end == value.c_str() ? fallback : static_cast<int>(parsed);
}

std::vector<GroupPair> tokenize(const std::string& text) {
    std::vector<GroupPair> pairs;
    std::istringstream stream(text);
    std::string codeLine;
    std::string valueLine;
    while (std::getline(stream, codeLine) && std::getline(stream, valueLine)) {
        pairs.push_back({toInt(trim(codeLine)), trim(valueLine)});
    }
    return pairs;
}

void addPartFromPoints(std::vector<Part>& parts, std::wstring name, std::vector<Vec2> points, bool close) {
    if (points.size() < 2) {
        return;
    }
    if (close && points.size() > 2 && !almostEqual(points.front(), points.back())) {
        points.push_back(points.front());
    }

    Ring ring;
    ring.points = std::move(points);
    ring.winding = windingDirection(ring.points);

    Part part;
    part.name = std::move(name);
    part.rings.push_back(std::move(ring));
    part.updateDerivedGeometry();
    parts.push_back(std::move(part));
}

size_t nextEntity(const std::vector<GroupPair>& pairs, size_t start) {
    size_t i = start;
    while (i < pairs.size() && pairs[i].code != 0) {
        ++i;
    }
    return i;
}

} // namespace

ImportResult DxfParser::parseFile(const std::wstring& path, double flattenTolerance) const {
    const auto text = readTextFile(path);
    if (text.empty()) {
        return {false, L"DXF file could not be read", {}};
    }
    return parseText(text, flattenTolerance);
}

ImportResult DxfParser::parseText(const std::string& text, double flattenTolerance) const {
    ImportResult result;
    result.ok = true;
    const auto pairs = tokenize(text);
    size_t counter = 1;

    for (size_t i = 0; i < pairs.size();) {
        if (pairs[i].code != 0) {
            ++i;
            continue;
        }

        const std::string entity = upper(pairs[i].value);
        if (entity == "LINE") {
            const size_t end = nextEntity(pairs, i + 1);
            double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
            for (size_t j = i + 1; j < end; ++j) {
                if (pairs[j].code == 10) x1 = toDouble(pairs[j].value);
                if (pairs[j].code == 20) y1 = toDouble(pairs[j].value);
                if (pairs[j].code == 11) x2 = toDouble(pairs[j].value);
                if (pairs[j].code == 21) y2 = toDouble(pairs[j].value);
            }
            addPartFromPoints(result.parts, L"dxf-line-" + std::to_wstring(counter++), {{x1, y1}, {x2, y2}}, false);
            i = end;
        } else if (entity == "LWPOLYLINE") {
            const size_t end = nextEntity(pairs, i + 1);
            std::vector<Vec2> points;
            bool closed = false;
            double pendingX = 0.0;
            bool hasX = false;
            for (size_t j = i + 1; j < end; ++j) {
                if (pairs[j].code == 70) {
                    closed = (toInt(pairs[j].value) & 1) != 0;
                } else if (pairs[j].code == 10) {
                    pendingX = toDouble(pairs[j].value);
                    hasX = true;
                } else if (pairs[j].code == 20 && hasX) {
                    points.push_back({pendingX, toDouble(pairs[j].value)});
                    hasX = false;
                }
            }
            addPartFromPoints(result.parts, L"dxf-polyline-" + std::to_wstring(counter++), std::move(points), closed);
            i = end;
        } else if (entity == "POLYLINE") {
            std::vector<Vec2> points;
            bool closed = false;
            ++i;
            while (i < pairs.size()) {
                if (pairs[i].code == 0 && upper(pairs[i].value) == "SEQEND") {
                    ++i;
                    break;
                }
                if (pairs[i].code == 0 && upper(pairs[i].value) == "VERTEX") {
                    const size_t end = nextEntity(pairs, i + 1);
                    double x = 0.0, y = 0.0;
                    for (size_t j = i + 1; j < end; ++j) {
                        if (pairs[j].code == 10) x = toDouble(pairs[j].value);
                        if (pairs[j].code == 20) y = toDouble(pairs[j].value);
                        if (pairs[j].code == 70) closed = (toInt(pairs[j].value) & 1) != 0;
                    }
                    points.push_back({x, y});
                    i = end;
                } else {
                    ++i;
                }
            }
            addPartFromPoints(result.parts, L"dxf-polyline-" + std::to_wstring(counter++), std::move(points), closed);
        } else if (entity == "CIRCLE" || entity == "ARC") {
            const size_t end = nextEntity(pairs, i + 1);
            double cx = 0.0, cy = 0.0, radius = 0.0, startDeg = 0.0, endDeg = 360.0;
            for (size_t j = i + 1; j < end; ++j) {
                if (pairs[j].code == 10) cx = toDouble(pairs[j].value);
                if (pairs[j].code == 20) cy = toDouble(pairs[j].value);
                if (pairs[j].code == 40) radius = toDouble(pairs[j].value);
                if (pairs[j].code == 50) startDeg = toDouble(pairs[j].value);
                if (pairs[j].code == 51) endDeg = toDouble(pairs[j].value);
            }
            if (radius > 0.0) {
                double sweep = degreesToRadians(endDeg - startDeg);
                if (entity == "ARC" && sweep <= 0.0) {
                    sweep += twoPi;
                }
                auto points = flattenCircularArc({cx, cy}, radius, radius, degreesToRadians(startDeg), entity == "CIRCLE" ? twoPi : sweep, flattenTolerance);
                addPartFromPoints(result.parts, L"dxf-arc-" + std::to_wstring(counter++), std::move(points), entity == "CIRCLE");
            }
            i = end;
        } else {
            ++i;
        }
    }

    if (result.parts.empty()) {
        result.ok = false;
        result.message = L"No supported DXF geometry was found";
    } else {
        result.message = L"DXF imported";
    }
    return result;
}

} // namespace nest
