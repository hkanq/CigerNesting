#pragma once

#include "bridge/session_format.h"
#include "core/document.h"
#include "engine/solver_state.h"
#include <string>

namespace nest {

class CorelBridge {
public:
    bool isAvailable() const;
    SessionFormat createSessionFromDocument(const Document& document) const;
    std::wstring exportResultSession(const Document& document, const SolverResult& result) const;
};

} // namespace nest
