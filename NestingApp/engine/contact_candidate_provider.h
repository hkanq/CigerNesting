#pragma once

#include "core/document.h"
#include "engine/analytic_contact_candidate.h"
#include "engine/engine_settings.h"
#include <vector>

namespace nest {

using ContactCandidate = AnalyticContactCandidate;
using ContactCandidateRequest = AnalyticContactRequest;
using ContactCandidateStats = AnalyticContactStats;

class IContactCandidateProvider {
public:
    virtual ~IContactCandidateProvider() = default;

    virtual std::vector<ContactCandidate> generatePartPartCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const = 0;

    virtual std::vector<ContactCandidate> generatePartSheetCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const = 0;

    virtual std::vector<ContactCandidate> generatePartHoleCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const = 0;

    virtual std::vector<ContactCandidate> generateCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const = 0;
};

class AnalyticContactCandidateProvider final : public IContactCandidateProvider {
public:
    std::vector<ContactCandidate> generatePartPartCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const override {
        return generateCandidates(document, settings, poses, request, stats);
    }

    std::vector<ContactCandidate> generatePartSheetCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const override {
        return generateCandidates(document, settings, poses, request, stats);
    }

    std::vector<ContactCandidate> generatePartHoleCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const override {
        return generateCandidates(document, settings, poses, request, stats);
    }

    std::vector<ContactCandidate> generateCandidates(
        const Document& document,
        const EngineSettings& settings,
        const std::vector<Pose>& poses,
        const ContactCandidateRequest& request,
        ContactCandidateStats* stats = nullptr) const override {
        return AnalyticContactCandidateGenerator{}.generate(document, settings, poses, request, stats);
    }
};

} // namespace nest
