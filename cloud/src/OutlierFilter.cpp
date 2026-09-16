// Copyright by BeeX [2026]

#include <cloud/OutlierFilter.h>

#include <algorithm>
#include <cmath>

namespace cloud {
namespace {

// Running sums over a rectangle, one row and one column larger than the image.
class SummedArea {
public:
    SummedArea(int width, int height) : stride_(width + 1), sum_(static_cast<size_t>(width + 1) * (height + 1), 0.0) {}

    void add(int u, int v, double value) {
        const size_t i = index(u + 1, v + 1);
        sum_[i]        = value + sum_[i - 1] + sum_[i - stride_] - sum_[i - stride_ - 1];
    }

    // Sum over u0..u1 and v0..v1, inclusive.
    double over(int u0, int v0, int u1, int v1) const {
        return sum_[index(u1 + 1, v1 + 1)] - sum_[index(u1 + 1, v0)] - sum_[index(u0, v1 + 1)] + sum_[index(u0, v0)];
    }

private:
    size_t index(int u, int v) const { return static_cast<size_t>(v) * stride_ + u; }

    size_t              stride_;
    std::vector<double> sum_;
};

}  // namespace

OutlierFilter::OutlierFilter(const OutlierFilterSettings &settings, const DepthImage &depth)
        : width_(depth.width()), supported_(static_cast<size_t>(depth.width()) * depth.height(), 0) {
    const int width  = depth.width();
    const int height = depth.height();

    SummedArea count(width, height);
    SummedArea depth_sum(width, height);
    SummedArea depth_u_sum(width, height);
    SummedArea depth_v_sum(width, height);
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            const double z = depth.seen(u, v) ? depth.at(u, v) : 0.0;
            count.add(u, v, depth.seen(u, v) ? 1.0 : 0.0);
            depth_sum.add(u, v, z);
            depth_u_sum.add(u, v, z * u);
            depth_v_sum.add(u, v, z * v);
        }
    }

    // The local surface is a plane fitted over the window: depth changes by slope_u per column and slope_v per row.
    const int    half   = settings.slope_window_px / 2;
    const double spread = (settings.slope_window_px * settings.slope_window_px - 1) / 12.0;

    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            if (!depth.seen(u, v)) {
                continue;
            }
            const int    u0 = std::max(0, u - half);
            const int    v0 = std::max(0, v - half);
            const int    u1 = std::min(width - 1, u + half);
            const int    v1 = std::min(height - 1, v + half);
            const double n  = count.over(u0, v0, u1, v1);

            const double mean_depth = depth_sum.over(u0, v0, u1, v1) / n;
            const double slope_u    = (depth_u_sum.over(u0, v0, u1, v1) / n - u * mean_depth) / spread;
            const double slope_v    = (depth_v_sum.over(u0, v0, u1, v1) / n - v * mean_depth) / spread;
            const double centre     = depth.at(u, v);

            int agreeing = 0;
            for (int dv = -1; dv <= 1; ++dv) {
                for (int du = -1; du <= 1; ++du) {
                    const int nu = u + du;
                    const int nv = v + dv;
                    if ((du == 0 && dv == 0) || nu < 0 || nv < 0 || nu >= width || nv >= height || !depth.seen(nu, nv)) {
                        continue;
                    }
                    const double expected = centre + slope_u * du + slope_v * dv;
                    agreeing += std::fabs(depth.at(nu, nv) - expected) <= settings.depth_tolerance ? 1 : 0;
                }
            }
            supported_[static_cast<size_t>(v) * width_ + u] = agreeing >= settings.min_agreeing_neighbours ? 1 : 0;
        }
    }
}

}  // namespace cloud
