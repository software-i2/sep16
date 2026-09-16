// Copyright by BeeX [2026]

#include <cloud/CandidateAveraging.h>
#include <cloud/ConsensusAveraging.h>

#include <stdexcept>

namespace cloud {

// Add a new method here and give it a section under candidate_averaging in cloud.yaml.
std::unique_ptr<CandidateAveraging> makeCandidateAveraging(const CloudConfig &config) {
    if (config.averaging_method == "consensus") {
        return std::unique_ptr<CandidateAveraging>(new ConsensusAveraging(config.consensus));
    }
    throw std::invalid_argument("unknown candidate_averaging/method '" + config.averaging_method + "'");
}

}  // namespace cloud
