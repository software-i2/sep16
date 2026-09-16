// Copyright by BeeX [2026]

#ifndef CLOUD_OUTLIERFILTER_H
#define CLOUD_OUTLIERFILTER_H

#include <cloud/DepthImage.h>

#include <cstdint>
#include <vector>

namespace cloud {

// Flying pixel test: a pixel is supported when enough of its 8 neighbours lie on the same local surface.
class OutlierFilter {
public:
    OutlierFilter(const OutlierFilterSettings &settings, const DepthImage &depth);

    bool supported(int u, int v) const { return supported_[static_cast<size_t>(v) * width_ + u] != 0; }

private:
    int                  width_;
    std::vector<uint8_t> supported_;
};

}  // namespace cloud

#endif  // CLOUD_OUTLIERFILTER_H
