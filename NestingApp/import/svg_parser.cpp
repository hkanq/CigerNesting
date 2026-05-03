#include "import/svg_parser.h"

#include "core/math_utils.h"
#include "geometry/arc.h"
#include "geometry/bezier.h"
#include "geometry/flatten.h"
#include "geometry/polygon_utils.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
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

std::wstring widenAscii(const std::string& text) {
    return {text.begin(), text.end()};
}

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

void skipWhitespace(const std::string& text, size_t& pos) {
    while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) {
        ++pos;
    }
}

void skipSeparators(const std::string& text, size_t& pos) {
    while (pos < text.size()) {
        const char ch = text[pos];
        if (std::isspace(static_cast<unsigned char>(ch)) || ch == ',') {
            ++pos;
        } else {
            break;
        }
    }
}

bool hasNumberAhead(const std::string& text, size_t pos) {
    skipSeparators(text, pos);
    if (pos >= text.size()) {
        return false;
    }
    const char ch = text[pos];
    return std::isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+' || ch == '.';
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

std::vector<double> parseNumbers(const std::string& text) {
    std::vector<double> values;
    size_t pos = 0;
    while (pos < text.size()) {
        double value = 0.0;
        if (readNumber(text, pos, value)) {
            values.push_back(value);
        } else {
            ++pos;
        }
    }
    return values;
}

std::map<std::string, std::string> parseAttributes(const std::string& tag) {
    std::map<std::string, std::string> attrs;
    size_t pos = 0;
    skipWhitespace(tag, pos);
    while (pos < tag.size() && !std::isspace(static_cast<unsigned char>(tag[pos])) && tag[pos] != '/') {
        ++pos;
    }

    while (pos < tag.size()) {
        skipWhitespace(tag, pos);
        if (pos >= tag.size() || tag[pos] == '/') {
            break;
        }

        const size_t keyStart = pos;
        while (pos < tag.size()) {
            const char ch = tag[pos];
            if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '-' || ch == ':' || ch == '.') {
                ++pos;
            } else {
                break;
            }
        }
        if (pos == keyStart) {
            ++pos;
            continue;
        }
        std::string key = toLower(tag.substr(keyStart, pos - keyStart));
        skipWhitespace(tag, pos);
        if (pos >= tag.size() || tag[pos] != '=') {
            attrs[key] = {};
            continue;
        }
        ++pos;
        skipWhitespace(tag, pos);
        if (pos >= tag.size()) {
            attrs[key] = {};
            break;
        }

        std::string value;
        if (tag[pos] == '"' || tag[pos] == '\'') {
            const char quote = tag[pos++];
            const size_t valueStart = pos;
            while (pos < tag.size() && tag[pos] != quote) {
                ++pos;
            }
            value = tag.substr(valueStart, pos - valueStart);
            if (pos < tag.size()) {
                ++pos;
            }
        } else {
            const size_t valueStart = pos;
            while (pos < tag.size() && !std::isspace(static_cast<unsigned char>(tag[pos])) && tag[pos] != '/') {
                ++pos;
            }
            value = tag.substr(valueStart, pos - valueStart);
        }
        attrs[key] = value;
    }
    return attrs;
}

std::string tagName(const std::string& tag) {
    size_t pos = 0;
    skipWhitespace(tag, pos);
    if (pos < tag.size() && tag[pos] == '/') {
        ++pos;
    }
    const size_t start = pos;
    while (pos < tag.size() && !std::isspace(static_cast<unsigned char>(tag[pos])) && tag[pos] != '/' && tag[pos] != '>') {
        ++pos;
    }
    return toLower(tag.substr(start, pos - start));
}

std::string attr(const std::map<std::string, std::string>& attrs, const std::string& key, const std::string& fallback = {}) {
    const auto it = attrs.find(key);
    return it == attrs.end() ? fallback : it->second;
}

double attrNumber(const std::map<std::string, std::string>& attrs, const std::string& key, double fallback = 0.0) {
    const auto it = attrs.find(key);
    if (it == attrs.end()) {
        return fallback;
    }
    size_t pos = 0;
    double value = fallback;
    return readNumber(it->second, pos, value) ? value : fallback;
}

Transform parseTransform(const std::string& text) {
    Transform result = Transform::identity();
    size_t pos = 0;
    while (pos < text.size()) {
        skipWhitespace(text, pos);
        const size_t nameStart = pos;
        while (pos < text.size() && std::isalpha(static_cast<unsigned char>(text[pos]))) {
            ++pos;
        }
        if (pos == nameStart) {
            ++pos;
            continue;
        }
        const std::string name = toLower(text.substr(nameStart, pos - nameStart));
        skipWhitespace(text, pos);
        if (pos >= text.size() || text[pos] != '(') {
            continue;
        }
        ++pos;
        const size_t argsStart = pos;
        int depth = 1;
        while (pos < text.size() && depth > 0) {
            if (text[pos] == '(') {
                ++depth;
            } else if (text[pos] == ')') {
                --depth;
                if (depth == 0) {
                    break;
                }
            }
            ++pos;
        }
        const std::string args = text.substr(argsStart, pos - argsStart);
        if (pos < text.size() && text[pos] == ')') {
            ++pos;
        }

        const auto values = parseNumbers(args);
        Transform next = Transform::identity();
        if (name == "translate" && !values.empty()) {
            next = Transform::translated(values[0], values.size() > 1 ? values[1] : 0.0);
        } else if (name == "scale" && !values.empty()) {
            next = Transform::scaled(values[0], values.size() > 1 ? values[1] : values[0]);
        } else if (name == "rotate" && !values.empty()) {
            const double radians = degreesToRadians(values[0]);
            if (values.size() >= 3) {
                next = Transform::translated(values[1], values[2]) * Transform::rotated(radians) * Transform::translated(-values[1], -values[2]);
            } else {
                next = Transform::rotated(radians);
            }
        }
        result = result * next;
    }
    return result;
}

Path parsePathData(const std::string& d, double tolerance) {
    (void)tolerance;
    Path path;
    size_t pos = 0;
    char command = 0;
    Vec2 current{};
    Vec2 subpathStart{};

    while (pos < d.size()) {
        skipSeparators(d, pos);
        if (pos >= d.size()) {
            break;
        }
        if (std::isalpha(static_cast<unsigned char>(d[pos]))) {
            command = d[pos++];
        } else if (command == 0) {
            ++pos;
            continue;
        }

        const bool relative = std::islower(static_cast<unsigned char>(command)) != 0;
        const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(command)));

        if (upper == 'Z') {
            path.close();
            current = subpathStart;
            command = 0;
            continue;
        }

        if (upper == 'M') {
            bool first = true;
            while (hasNumberAhead(d, pos)) {
                double x = 0.0;
                double y = 0.0;
                if (!readNumber(d, pos, x) || !readNumber(d, pos, y)) {
                    break;
                }
                Vec2 p{x, y};
                if (relative) {
                    p += current;
                }
                if (first) {
                    path.moveTo(p);
                    subpathStart = p;
                    first = false;
                } else {
                    path.lineTo(p);
                }
                current = p;
            }
            command = relative ? 'l' : 'L';
            continue;
        }

        if (upper == 'L') {
            while (hasNumberAhead(d, pos)) {
                double x = 0.0;
                double y = 0.0;
                if (!readNumber(d, pos, x) || !readNumber(d, pos, y)) {
                    break;
                }
                Vec2 p{x, y};
                if (relative) {
                    p += current;
                }
                path.lineTo(p);
                current = p;
            }
        } else if (upper == 'H') {
            while (hasNumberAhead(d, pos)) {
                double x = 0.0;
                if (!readNumber(d, pos, x)) {
                    break;
                }
                Vec2 p{relative ? current.x + x : x, current.y};
                path.lineTo(p);
                current = p;
            }
        } else if (upper == 'V') {
            while (hasNumberAhead(d, pos)) {
                double y = 0.0;
                if (!readNumber(d, pos, y)) {
                    break;
                }
                Vec2 p{current.x, relative ? current.y + y : y};
                path.lineTo(p);
                current = p;
            }
        } else if (upper == 'C') {
            while (hasNumberAhead(d, pos)) {
                double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0, x = 0.0, y = 0.0;
                if (!readNumber(d, pos, x1) || !readNumber(d, pos, y1) || !readNumber(d, pos, x2) || !readNumber(d, pos, y2) || !readNumber(d, pos, x) || !readNumber(d, pos, y)) {
                    break;
                }
                Vec2 c1{x1, y1};
                Vec2 c2{x2, y2};
                Vec2 p{x, y};
                if (relative) {
                    c1 += current;
                    c2 += current;
                    p += current;
                }
                path.cubicTo(c1, c2, p);
                current = p;
            }
        } else if (upper == 'Q') {
            while (hasNumberAhead(d, pos)) {
                double x1 = 0.0, y1 = 0.0, x = 0.0, y = 0.0;
                if (!readNumber(d, pos, x1) || !readNumber(d, pos, y1) || !readNumber(d, pos, x) || !readNumber(d, pos, y)) {
                    break;
                }
                Vec2 c{x1, y1};
                Vec2 p{x, y};
                if (relative) {
                    c += current;
                    p += current;
                }
                path.quadraticTo(c, p);
                current = p;
            }
        } else if (upper == 'A') {
            while (hasNumberAhead(d, pos)) {
                double rx = 0.0, ry = 0.0, rot = 0.0, largeArc = 0.0, sweep = 0.0, x = 0.0, y = 0.0;
                if (!readNumber(d, pos, rx) || !readNumber(d, pos, ry) || !readNumber(d, pos, rot) || !readNumber(d, pos, largeArc) || !readNumber(d, pos, sweep) || !readNumber(d, pos, x) || !readNumber(d, pos, y)) {
                    break;
                }
                (void)rx; (void)ry; (void)rot; (void)largeArc; (void)sweep;
                Vec2 p{x, y};
                if (relative) {
                    p += current;
                }
                // TODO: Replace this line fallback with full SVG elliptical arc conversion.
                path.lineTo(p);
                current = p;
            }
        } else {
            ++pos;
        }
    }

    return path;
}

Part makePart(std::wstring name, std::vector<Ring> rings, const Transform& transform) {
    classifyHoleRings(rings);
    for (auto& ring : rings) {
        for (auto& point : ring.points) {
            point = transform.apply(point);
        }
        ring.winding = windingDirection(ring.points);
    }

    Part part;
    part.name = std::move(name);
    part.rings = std::move(rings);
    part.updateDerivedGeometry();
    return part;
}

Ring ringFromPoints(const std::vector<Vec2>& points, bool close) {
    Ring ring;
    ring.points = points;
    if (close && ring.points.size() > 2 && !almostEqual(ring.points.front(), ring.points.back())) {
        ring.points.push_back(ring.points.front());
    }
    ring.winding = windingDirection(ring.points);
    return ring;
}

std::vector<Vec2> parsePointList(const std::string& text) {
    const auto values = parseNumbers(text);
    std::vector<Vec2> points;
    points.reserve(values.size() / 2);
    for (size_t i = 0; i + 1 < values.size(); i += 2) {
        points.push_back({values[i], values[i + 1]});
    }
    return points;
}

} // namespace

ImportResult SvgParser::parseFile(const std::wstring& path, double flattenTolerance) const {
    const std::string text = readTextFile(path);
    if (text.empty()) {
        return {false, L"SVG file could not be read", {}};
    }
    return parseText(text, flattenTolerance);
}

ImportResult SvgParser::parseText(const std::string& text, double flattenTolerance) const {
    ImportResult result;
    result.ok = true;

    std::vector<Transform> transformStack;
    transformStack.push_back(Transform::identity());

    size_t pos = 0;
    size_t partCounter = 1;
    while (true) {
        const size_t open = text.find('<', pos);
        if (open == std::string::npos) {
            break;
        }
        const size_t close = text.find('>', open + 1);
        if (close == std::string::npos) {
            break;
        }

        std::string tag = text.substr(open + 1, close - open - 1);
        pos = close + 1;
        if (tag.empty() || tag[0] == '?' || tag[0] == '!') {
            continue;
        }

        const bool closing = tag[0] == '/';
        const bool selfClosing = !tag.empty() && tag.back() == '/';
        const std::string name = tagName(tag);

        if (closing) {
            if (name == "g" && transformStack.size() > 1) {
                transformStack.pop_back();
            }
            continue;
        }

        const auto attrs = parseAttributes(tag);
        const Transform elementTransform = transformStack.back() * parseTransform(attr(attrs, "transform"));

        if (name == "g") {
            if (!selfClosing) {
                transformStack.push_back(elementTransform);
            }
            continue;
        }

        std::vector<Ring> rings;
        if (name == "path") {
            const std::string d = attr(attrs, "d");
            if (!d.empty()) {
                rings = flattenPathToRings(parsePathData(d, flattenTolerance), flattenTolerance);
            }
        } else if (name == "polygon" || name == "polyline") {
            auto points = parsePointList(attr(attrs, "points"));
            if (points.size() >= 2) {
                rings.push_back(ringFromPoints(points, name == "polygon"));
            }
        } else if (name == "rect") {
            const double x = attrNumber(attrs, "x");
            const double y = attrNumber(attrs, "y");
            const double w = attrNumber(attrs, "width");
            const double h = attrNumber(attrs, "height");
            if (w > 0.0 && h > 0.0) {
                rings.push_back(ringFromPoints({{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}}, true));
            }
        } else if (name == "circle") {
            const double cx = attrNumber(attrs, "cx");
            const double cy = attrNumber(attrs, "cy");
            const double r = attrNumber(attrs, "r");
            if (r > 0.0) {
                auto points = flattenCircularArc({cx, cy}, r, r, 0.0, twoPi, flattenTolerance);
                rings.push_back(ringFromPoints(points, true));
            }
        } else if (name == "ellipse") {
            const double cx = attrNumber(attrs, "cx");
            const double cy = attrNumber(attrs, "cy");
            const double rx = attrNumber(attrs, "rx");
            const double ry = attrNumber(attrs, "ry");
            if (rx > 0.0 && ry > 0.0) {
                auto points = flattenCircularArc({cx, cy}, rx, ry, 0.0, twoPi, flattenTolerance);
                rings.push_back(ringFromPoints(points, true));
            }
        }

        if (!rings.empty()) {
            Part part = makePart(L"svg-part-" + std::to_wstring(partCounter++), std::move(rings), elementTransform);
            if (!part.empty() && part.localBounds.isValid()) {
                result.parts.push_back(std::move(part));
            }
        }
    }

    if (result.parts.empty()) {
        result.ok = false;
        result.message = L"No supported SVG geometry was found";
    } else {
        result.message = L"SVG imported";
    }

    return result;
}

} // namespace nest
