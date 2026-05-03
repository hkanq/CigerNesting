#include "bridge/corel_bridge.h"

#include <algorithm>

namespace nest {

bool CorelBridge::isAvailable() const {
    return false;
}

SessionFormat CorelBridge::createSessionFromDocument(const Document& document) const {
    SessionFormat session;
    session.sourceApplication = L"CigerNesting";
    session.documentId = document.sourcePath;
    session.parts.reserve(document.parts.size());
    for (size_t i = 0; i < document.parts.size(); ++i) {
        SessionPartRecord record;
        record.externalId = L"part-" + std::to_wstring(i + 1);
        record.name = document.parts[i].name;
        record.sourcePose = document.parts[i].pose;
        record.resultPose = document.parts[i].pose;
        session.parts.push_back(record);
    }
    return session;
}

std::wstring CorelBridge::exportResultSession(const Document& document, const SolverResult& result) const {
    SessionFormat session = createSessionFromDocument(document);
    const size_t count = std::min(session.parts.size(), result.bestPoses.size());
    for (size_t i = 0; i < count; ++i) {
        session.parts[i].resultPose = result.bestPoses[i];
    }
    return serializeSession(session);
}

} // namespace nest
