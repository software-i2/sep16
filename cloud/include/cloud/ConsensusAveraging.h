// Copyright by BeeX [2026]

#ifndef CLOUD_CONSENSUSAVERAGING_H
#define CLOUD_CONSENSUSAVERAGING_H

#include <cloud/CandidateAveraging.h>

namespace cloud {

// Keeps the spots on the bar that enough frames agree on, averaged; nothing if the scene drifted.
class ConsensusAveraging : public CandidateAveraging {
public:
    explicit ConsensusAveraging(const ConsensusSettings &settings) : settings_(settings) {}

    AveragedCandidates average(const std::vector<std::vector<GraspPose>> &frames) const override;

private:
    ConsensusSettings settings_;
};

}  // namespace cloud

#endif  // CLOUD_CONSENSUSAVERAGING_H
