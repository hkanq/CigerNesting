#pragma once

#include "core/polygon.h"
#include <string>
#include <vector>

namespace nest {

struct SessionPartRecord {
    std::wstring externalId;
    std::wstring name;
    Pose sourcePose;
    Pose resultPose;
};

struct SessionFormat {
    std::wstring sourceApplication;
    std::wstring documentId;
    std::vector<SessionPartRecord> parts;
};

std::wstring serializeSession(const SessionFormat& session);
SessionFormat deserializeSession(const std::wstring& text);

} // namespace nest
