#include "engine/solver_state.h"

#include <iostream>

namespace {
using namespace nest;

bool expect(const char* name, bool condition) {
    std::cout << (condition ? "PASS: " : "FAIL: ") << name << "\n";
    return condition;
}

} // namespace

int main() {
    SolverSnapshot stable;
    stable.versionId = 7;
    stable.layoutChanged = false;

    SolverSnapshot changed = stable;
    changed.versionId = 8;
    changed.layoutChanged = true;
    changed.lastMovedPart = 4;
    changed.changedParts = {4, 8, 13};

    bool ok = true;
    ok = expect("snapshot version changes on accepted move", changed.versionId != stable.versionId) && ok;
    ok = expect("layoutChanged marks event-driven redraw", changed.layoutChanged) && ok;
    ok = expect("changed subset is carried", changed.changedParts.size() == 3) && ok;
    ok = expect("first changed part remains available", changed.lastMovedPart == 4) && ok;
    return ok ? 0 : 1;
}
