#pragma once

#include "engine/contact_candidate_provider.h"

namespace nest {

class InnerFitCandidateProvider final : public IContactCandidateProvider {
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
};

} // namespace nest
