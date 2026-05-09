#pragma once

#include "engine/contact_candidate_provider.h"
#include "engine/nfp_cache.h"
#include "engine/nfp_solver_cache.h"

namespace nest {

class NfpCandidateProvider final : public IContactCandidateProvider {
public:
    std::vector<ContactCandidate> generatePartPartCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const override;

    std::vector<ContactCandidate> generatePartSheetCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const override;

    std::vector<ContactCandidate> generatePartHoleCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const override;

    std::vector<ContactCandidate> generateCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const override;

    const NfpCache& cache() const { return cache_; }

private:
    mutable NfpCache cache_;
    mutable NfpSolverCache solverCache_;
};

} // namespace nest


