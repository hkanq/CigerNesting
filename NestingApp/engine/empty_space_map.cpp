#include "engine/empty_space_map.h"

#include "geometry/collision.h"
#include "geometry/transformed_shape.h"
#include <algorithm>
#include <cmath>
#include <queue>

namespace nest {
namespace {

AABB sheetBounds(const Document& document, const EngineSettings& settings) {
    const double width = document.sheet.width > 0.0 ? document.sheet.width : settings.sheetWidth;
    const double height = document.sheet.height > 0.0 ? document.sheet.height : settings.sheetHeight;
    return AABB::fromMinMax(
        {document.sheet.origin.x + settings.margin, document.sheet.origin.y + settings.margin},
        {document.sheet.origin.x + width - settings.margin, document.sheet.origin.y + height - settings.margin});
}

bool insideAnyPart(Vec2 point, const std::vector<TransformedPart>& parts) {
    for (const TransformedPart& part : parts) {
        if (!part.bounds.isValid() ||
            point.x < part.bounds.min.x || point.x > part.bounds.max.x ||
            point.y < part.bounds.min.y || point.y > part.bounds.max.y) {
            continue;
        }
        if (pointInSolidArea(part, point, 1e-6)) {
            return true;
        }
    }
    return false;
}

} // namespace

size_t EmptySpaceMap::fillableRegionCount(double minArea) const {
    return static_cast<size_t>(std::count_if(regions.begin(), regions.end(), [&](const EmptyRegion& region) {
        return region.area >= minArea;
    }));
}

EmptySpaceMap EmptySpaceAnalyzer::analyze(
    const Document& document,
    const EngineSettings& settings,
    const LayoutState& state,
    int requestedColumns,
    int requestedRows) const {
    EmptySpaceMap map;
    if (state.poses.empty() || document.parts.empty()) {
        return map;
    }

    std::vector<TransformedPart> transformed;
    transformed.reserve(std::min(document.parts.size(), state.poses.size()));
    for (size_t i = 0; i < document.parts.size() && i < state.poses.size(); ++i) {
        transformed.push_back(transformPart(document.parts[i], state.poses[i], static_cast<int>(i)));
        map.usedBounds.include(transformed.back().bounds);
    }
    if (!map.usedBounds.isValid()) {
        return map;
    }

    const AABB usable = sheetBounds(document, settings);
    AABB analysisBounds = map.usedBounds;
    analysisBounds = AABB::fromMinMax(
        {std::max(usable.min.x, analysisBounds.min.x), std::max(usable.min.y, analysisBounds.min.y)},
        {std::min(usable.max.x, analysisBounds.max.x), std::min(usable.max.y, analysisBounds.max.y)});
    if (!analysisBounds.isValid() || analysisBounds.width() <= 1e-6 || analysisBounds.height() <= 1e-6) {
        return map;
    }

    const int defaultColumns = document.parts.size() > 250 ? 72 : 56;
    const int defaultRows = document.parts.size() > 250 ? 48 : 38;
    map.columns = std::max(8, requestedColumns > 0 ? requestedColumns : defaultColumns);
    map.rows = std::max(8, requestedRows > 0 ? requestedRows : defaultRows);
    map.cellWidth = analysisBounds.width() / static_cast<double>(map.columns);
    map.cellHeight = analysisBounds.height() / static_cast<double>(map.rows);
    const double cellArea = map.cellWidth * map.cellHeight;

    std::vector<unsigned char> freeCells(static_cast<size_t>(map.columns * map.rows), 0);
    auto indexOf = [&](int x, int y) {
        return static_cast<size_t>(y * map.columns + x);
    };
    for (int y = 0; y < map.rows; ++y) {
        for (int x = 0; x < map.columns; ++x) {
            const Vec2 center{
                analysisBounds.min.x + (static_cast<double>(x) + 0.5) * map.cellWidth,
                analysisBounds.min.y + (static_cast<double>(y) + 0.5) * map.cellHeight
            };
            if (!insideAnyPart(center, transformed)) {
                freeCells[indexOf(x, y)] = 1;
                map.totalEmptyArea += cellArea;
            }
        }
    }

    std::vector<unsigned char> visited(freeCells.size(), 0);
    const int dx[] = {1, -1, 0, 0};
    const int dy[] = {0, 0, 1, -1};
    for (int y = 0; y < map.rows; ++y) {
        for (int x = 0; x < map.columns; ++x) {
            const size_t start = indexOf(x, y);
            if (!freeCells[start] || visited[start]) {
                continue;
            }
            EmptyRegion region;
            int minX = x;
            int maxX = x;
            int minY = y;
            int maxY = y;
            std::queue<std::pair<int, int>> pending;
            pending.push({x, y});
            visited[start] = 1;
            while (!pending.empty()) {
                const auto [cx, cy] = pending.front();
                pending.pop();
                ++region.cellCount;
                minX = std::min(minX, cx);
                maxX = std::max(maxX, cx);
                minY = std::min(minY, cy);
                maxY = std::max(maxY, cy);
                if (cx == 0 || cy == 0 || cx == map.columns - 1 || cy == map.rows - 1) {
                    region.touchesSheetBoundary = true;
                }
                for (int i = 0; i < 4; ++i) {
                    const int nx = cx + dx[i];
                    const int ny = cy + dy[i];
                    if (nx < 0 || ny < 0 || nx >= map.columns || ny >= map.rows) {
                        continue;
                    }
                    const size_t next = indexOf(nx, ny);
                    if (freeCells[next] && !visited[next]) {
                        visited[next] = 1;
                        pending.push({nx, ny});
                    }
                }
            }
            region.area = static_cast<double>(region.cellCount) * cellArea;
            region.bounds = AABB::fromMinMax(
                {analysisBounds.min.x + static_cast<double>(minX) * map.cellWidth,
                 analysisBounds.min.y + static_cast<double>(minY) * map.cellHeight},
                {analysisBounds.min.x + static_cast<double>(maxX + 1) * map.cellWidth,
                 analysisBounds.min.y + static_cast<double>(maxY + 1) * map.cellHeight});
            region.center = region.bounds.center();
            region.touchesUsedBoundary =
                std::abs(region.bounds.min.x - map.usedBounds.min.x) <= map.cellWidth ||
                std::abs(region.bounds.max.x - map.usedBounds.max.x) <= map.cellWidth ||
                std::abs(region.bounds.min.y - map.usedBounds.min.y) <= map.cellHeight ||
                std::abs(region.bounds.max.y - map.usedBounds.max.y) <= map.cellHeight;
            map.largestRegionArea = std::max(map.largestRegionArea, region.area);
            map.regions.push_back(region);
        }
    }
    std::stable_sort(map.regions.begin(), map.regions.end(), [](const EmptyRegion& a, const EmptyRegion& b) {
        return a.area > b.area;
    });
    return map;
}

} // namespace nest
