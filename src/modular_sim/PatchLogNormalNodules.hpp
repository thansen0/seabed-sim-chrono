#pragma once

#include <vector>
#include <string>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <unordered_map>
#include <utility>
#include <algorithm>

#include "AbstractNoduleGenerator.hpp"
#include "DynamicSystemMulticore.hpp"

class PatchLotNormalNodules : public AbstractNoduleGenerator {
private:
    // ---------- Lognormal diameter model ----------
    struct LogNormalDiam {
        // Diameter d ~ LogNormal(mu, sigma), i.e. ln(d) ~ Normal(mu, sigma)
        double mu = std::log(0.02); // default ~2 cm
        double sigma = 0.4;

        static LogNormalDiam from_median_sigma(double median_m, double sigma_) {
            LogNormalDiam L;
            L.mu = std::log(std::max(median_m, 1e-9));
            L.sigma = std::max(sigma_, 1e-9);
            return L;
        }

        // Fit from mean and p90 (90th percentile) of diameter.
        // mean = exp(mu + 0.5*sigma^2)
        // p90  = exp(mu + z90*sigma), z90 ~ 1.28155
        static LogNormalDiam from_mean_p90(double mean_m, double p90_m) {
            const double z90 = 1.281551565545;
            mean_m = std::max(mean_m, 1e-9);
            p90_m  = std::max(p90_m,  1e-9);

            // Solve: ln(p90/mean) = z90*sigma - 0.5*sigma^2
            const double a = 0.5;
            const double b = -z90;
            const double c = std::log(p90_m / mean_m);

            const double disc = b*b - 4*a*c;
            LogNormalDiam L;

            if (disc <= 0.0) {
                // Fallback if inputs are inconsistent
                L.mu = std::log(mean_m);
                L.sigma = 0.3;
                return L;
            }

            const double sqrt_disc = std::sqrt(disc);
            // Two roots; choose the smaller positive root typically.
            const double s1 = (-b - sqrt_disc) / (2*a);
            const double s2 = (-b + sqrt_disc) / (2*a);

            double sigma_ = 0.0;
            if (s1 > 0.0 && s2 > 0.0) sigma_ = std::min(s1, s2);
            else sigma_ = std::max(s1, s2);
            sigma_ = std::clamp(sigma_, 1e-6, 3.0);

            const double mu_ = std::log(mean_m) - 0.5 * sigma_ * sigma_;

            L.mu = mu_;
            L.sigma = sigma_;
            return L;
        }

        template <class URNG>
        double sample(URNG& rng) const {
            std::lognormal_distribution<double> dist(mu, sigma);
            return dist(rng);
        }

        // E[ area ] where area is projected disk area pi*(d/2)^2
        double expected_projected_area() const {
            // If d~LogNormal(mu,s), E[d^2] = exp(2mu + 2s^2)
            const double Ed2 = std::exp(2.0*mu + 2.0*sigma*sigma);
            return M_PI * 0.25 * Ed2;
        }

        // A high quantile for sizing the spatial hash cell (avoid tiny cell => too many keys)
        double approx_quantile(double p) const {
            // crude normal quantile approximations for common values
            double z = 0.0;
            if      (p >= 0.999) z = 3.090232306;
            else if (p >= 0.995) z = 2.575829304;
            else if (p >= 0.99)  z = 2.326347874;
            else if (p >= 0.95)  z = 1.644853627;
            else if (p >= 0.90)  z = 1.281551566;
            else                 z = 0.0;
            return std::exp(mu + sigma * z);
        }
    };

    // ---------- Spatial hash grid for overlap checks ----------
    struct CellKey {
        int ix;
        int iy;
        bool operator==(const CellKey& o) const { return ix == o.ix && iy == o.iy; }
    };

    struct CellKeyHash {
        std::size_t operator()(const CellKey& k) const noexcept {
            // simple mix
            std::uint64_t x = static_cast<std::uint32_t>(k.ix);
            std::uint64_t y = static_cast<std::uint32_t>(k.iy);
            std::uint64_t h = (x << 32) ^ y;
            h ^= (h >> 33);
            h *= 0xff51afd7ed558ccdULL;
            h ^= (h >> 33);
            h *= 0xc4ceb9fe1a85ec53ULL;
            h ^= (h >> 33);
            return static_cast<std::size_t>(h);
        }
    };

    // ---------- Generator parameters ----------
    struct FieldParams {
        double L = 10.0;               // length (m)
        double W = 10.0;               // width  (m)

        // Choose ONE:
        bool use_target_cover = true;
        double target_cover = 0.064;   // fraction, e.g. 0.064 = 6.4%
        double density = 250.0;        // nodules / m^2 if use_target_cover == false

        // Size distribution:
        LogNormalDiam diam = LogNormalDiam::from_mean_p90(0.018, 0.025); // mean 1.8cm, p90 2.5cm

        // Hard-core overlap:
        double gap = 0.0;              // extra spacing (m), e.g. 0.001 for 1mm
        int max_attempts_per_nodule = 50;

        // Patchiness:
        bool patchy = true;
        double patch_cell = 1.0;       // meters (intensity grid cell size)
        double patch_sigma = 0.8;      // larger => more patchy (0 => homogeneous)
        int patch_smooth_iters = 3;

        std::uint64_t seed = 12345;
    };

    // Simple in-place box blur on a 2D grid stored row-major
    void box_blur(std::vector<double>& a, int nx, int ny);

    // values set in constructor
    FieldParams P;

public:
    PatchLotNormalNodules(const std::string& config_path, DynamicSystemMulticore *sys);

    std::vector<Nodule> generate_nodules() override;
};
