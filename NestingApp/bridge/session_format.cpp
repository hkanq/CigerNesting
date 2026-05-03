#include "bridge/session_format.h"

#include <sstream>

namespace nest {
namespace {

void appendPose(std::wostringstream& out, const Pose& pose) {
    out << L"{\"x\":" << pose.x
        << L",\"y\":" << pose.y
        << L",\"angleRadians\":" << pose.angleRadians
        << L",\"mirrored\":" << (pose.mirrored ? L"true" : L"false") << L"}";
}

} // namespace

std::wstring serializeSession(const SessionFormat& session) {
    std::wostringstream out;
    out << L"{\"sourceApplication\":\"" << session.sourceApplication << L"\",";
    out << L"\"documentId\":\"" << session.documentId << L"\",";
    out << L"\"parts\":[";
    for (size_t i = 0; i < session.parts.size(); ++i) {
        const auto& part = session.parts[i];
        if (i > 0) {
            out << L",";
        }
        out << L"{\"externalId\":\"" << part.externalId << L"\",";
        out << L"\"name\":\"" << part.name << L"\",";
        out << L"\"sourcePose\":";
        appendPose(out, part.sourcePose);
        out << L",\"resultPose\":";
        appendPose(out, part.resultPose);
        out << L"}";
    }
    out << L"]}";
    return out.str();
}

SessionFormat deserializeSession(const std::wstring& text) {
    (void)text;
    // TODO: Keep this deliberately small until the bridge protocol settles.
    return {};
}

} // namespace nest
