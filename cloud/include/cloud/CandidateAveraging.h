// Copyright by BeeX [2026]

#ifndef CLOUD_CANDIDATEAVERAGING_H
#define CLOUD_CANDIDATEAVERAGING_H

#include <cloud/Frame.h>

#include <memory>
#include <string>
#include <vector>

namespace cloud {

struct AveragedCandidates {
    std::vector<GraspPose> candidates;
    std::string            summary;
};

// Combines the candidates of several frames, oldest first, into one set.
class CandidateAveraging {
public:
    virtual ~CandidateAveraging() = default;

    virtual AveragedCandidates average(const std::vector<std::vector<GraspPose>> &frames) const = 0;
};

// The method named by candidate_averaging/method. Throws std::invalid_argument on an unknown name.
std::unique_ptr<CandidateAveraging> makeCandidateAveraging(const CloudConfig &config);

}  // namespace cloud

#endif  // CLOUD_CANDIDATEAVERAGING_H
