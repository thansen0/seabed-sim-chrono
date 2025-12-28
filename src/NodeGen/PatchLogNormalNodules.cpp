#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <unordered_map>
#include <utility>
#include <algorithm>

#include "PatchLogNormalNodules.hpp"

using namespace chrono;
using namespace chrono::vehicle;

PatchLogNormalNodules::PatchLogNormalNodules(const toml::table& config_tbl, DynamicSystemMulticore *sys) : AbstractNoduleGenerator(config_tbl, sys) {
    auto sys_tbl = config_tbl["NODULES"];

    if (auto v = sys_tbl["nodule_rand_seed"].value<uint32_t>()) {
        P.seed = *v;
    } else {
        P.seed = std::random_device{}();
        std::cerr << "Warning: nodule_rand_seed not set in config, using random value " << P.seed << std::endl;
    }

    // Manages how nodules are placed
    if (auto v = sys_tbl["use_target_cover"].value<bool>()) {
        P.use_target_cover = *v;
    } else {
        std::cerr << "Warning: use_target_cover not set in config, using default " << P.use_target_cover << std::endl;
    }

    if (P.use_target_cover) {
        // distributing nodules by target cover fraction (i.e. percentage area covered)
        if (auto v = sys_tbl["nodule_target_cover_fraction"].value<double>()) {
            P.target_cover = *v;
        } else {
            std::cerr << "Warning: nodule_target_cover_fraction not set in config, using default " << P.target_cover << std::endl;
        }
    } else {
        // distributing nodules by density
        if (auto v = sys_tbl["nodule_density_per_m_sqr"].value<double>()) {
            P.density = *v;
        } else {
            std::cerr << "Warning: nodule_density_per_m_sqr not set in config, using default " << P.density << std::endl;
        }
    }

    if (auto v = sys_tbl["gap_between_nodules"].value<double>()) {
        P.gap = *v;
    } else {
        std::cerr << "Warning: gap_between_nodules not set in config, using default " << P.gap << std::endl;
    }

    if (auto v = sys_tbl["max_attempts_per_nodule"].value<uint32_t>()) {
        P.max_attempts_per_nodule = *v;
    } else {
        std::cerr << "Warning: max_attempts_per_nodule not set in config, using default " << P.max_attempts_per_nodule << std::endl;
    }

    // Enable patchyness, i.e. spatially varying intensity
    if (auto v = sys_tbl["using_patchy"].value<bool>()) {
        P.using_patchy = *v;
    } else {
        std::cerr << "Warning: using_patchy not set in config, using default " << P.using_patchy << std::endl;
    }

    if (auto v = sys_tbl["patch_cell"].value<double>()) {
        P.patch_cell = *v;
    } else {
        std::cerr << "Warning: patch_cell not set in config, using default " << P.patch_cell << std::endl;
    }

    if (auto v = sys_tbl["patch_sigma"].value<double>()) {
        P.patch_sigma = *v;
    } else {
        std::cerr << "Warning: patch_sigma not set in config, using default " << P.patch_sigma << std::endl;
    }

    if (auto v = sys_tbl["patch_smooth_iters"].value<uint32_t>()) {
        P.patch_smooth_iters = *v;
    } else {
        std::cerr << "Warning: patch_smooth_iters not set in config, using default " << P.patch_smooth_iters << std::endl;
    }

    // set LogNormalDiam nodule size distribution
    double nodule_diameter_mean{0.018}, nodule_diameter_p90{0.025};
    if (auto v = sys_tbl["nodule_diameter_mean"].value<double>()) {
        nodule_diameter_mean = *v;
    } else {
        std::cerr << "Warning: nodule_diameter_mean not set in config, using default " << nodule_diameter_mean << std::endl;
    }
    if (auto v = sys_tbl["nodule_diameter_p90"].value<double>()) {
        nodule_diameter_p90 = *v;
    } else {
        std::cerr << "Warning: nodule_diameter_p90 not set in config, using default " << nodule_diameter_p90 << std::endl;
    }
    P.diam = LogNormalDiam::from_mean_p90(nodule_diameter_mean, nodule_diameter_p90);

    // used to calculate number of nodules
    // read in from global variables, but could also be read in from config, design choice I may change later
    P.L = sim_length;
    P.W = sim_width;
    
}

void PatchLogNormalNodules::box_blur(std::vector<double>& a, int nx, int ny) {
    std::vector<double> out(a.size(), 0.0);
    auto at = [&](int x, int y) -> double& { return a[y*nx + x]; };
    auto outat = [&](int x, int y) -> double& { return out[y*nx + x]; };

    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            double sum = 0.0;
            int cnt = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int xx = x + dx, yy = y + dy;
                    if (0 <= xx && xx < nx && 0 <= yy && yy < ny) {
                        sum += at(xx, yy);
                        cnt++;
                    }
                }
            }
            outat(x, y) = sum / std::max(cnt, 1);
        }
    }
    a.swap(out);
}

std::vector<Nodule> PatchLogNormalNodules::generate_nodules() {
    std::mt19937_64 rng(P.seed);
    std::uniform_real_distribution<double> U01(0.0, 1.0);

    const double area_patch = P.L * P.W;

    // Base intensity (nodules per m^2)
    double lambda = 0.0;
    if (P.use_target_cover) {
        const double Earea = P.diam.expected_projected_area();
        lambda = P.target_cover / std::max(Earea, 1e-12);
    } else {
        lambda = std::max(P.density, 0.0);
    }

    // If patchy, build a smooth random field over a grid and turn it into multipliers
    int nx = std::max(1, static_cast<int>(std::ceil(P.L / P.patch_cell)));
    int ny = std::max(1, static_cast<int>(std::ceil(P.W / P.patch_cell)));
    std::vector<double> field(nx * ny, 0.0);

    if (P.using_patchy && P.patch_sigma > 0.0) {
        std::normal_distribution<double> N01(0.0, 1.0);
        for (auto& v : field) v = N01(rng);
        for (int it = 0; it < P.patch_smooth_iters; ++it) box_blur(field, nx, ny);

        // Convert to positive multipliers (log-Gaussian), then normalize to mean 1
        double sum_mult = 0.0;
        for (auto& v : field) {
            v = std::exp(P.patch_sigma * v);
            sum_mult += v;
        }
        const double mean_mult = sum_mult / std::max<std::size_t>(field.size(), 1);
        for (auto& v : field) v /= std::max(mean_mult, 1e-12);
    } else {
        std::fill(field.begin(), field.end(), 1.0);
    }

    // Spatial hash for overlap checks
    const double cellSize = std::max(P.diam.approx_quantile(0.99), 0.005); // >= 5mm
    std::unordered_map<CellKey, std::vector<int>, CellKeyHash> grid;
    grid.reserve(4096);

    auto cell_of = [&](double x, double y) -> CellKey {
        return CellKey{
            static_cast<int>(std::floor(x / cellSize)),
            static_cast<int>(std::floor(y / cellSize))
        };
    };

    std::vector<Nodule> out;
    out.reserve(static_cast<std::size_t>(lambda * area_patch));

    auto ok_no_overlap = [&](double x, double y, double r) -> bool {
        CellKey c = cell_of(x, y);
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                CellKey cc{c.ix + dx, c.iy + dy};
                auto it = grid.find(cc);
                if (it == grid.end()) continue;
                for (int idx : it->second) {
                    const auto& n = out[static_cast<std::size_t>(idx)];
                    const double r2 = 0.5*n.d;
                    const double minDist = r + r2 + P.gap;
                    const double dx2 = x - n.x;
                    const double dy2 = y - n.y;
                    if (dx2*dx2 + dy2*dy2 < minDist*minDist) return false;
                }
            }
        }
        return true;
    };

    auto insert_grid = [&](int idx) {
        const auto& n = out[static_cast<std::size_t>(idx)];
        grid[cell_of(n.x, n.y)].push_back(idx);
    };

    // Per-cell generation
    for (int j = 0; j < ny; ++j) {
        const double y0 = j * P.patch_cell;
        const double y1 = std::min(P.W, (j + 1) * P.patch_cell);
        const double cellH = std::max(0.0, y1 - y0);

        for (int i = 0; i < nx; ++i) {
            const double x0 = i * P.patch_cell;
            const double x1 = std::min(P.L, (i + 1) * P.patch_cell);
            const double cellW = std::max(0.0, x1 - x0);

            const double cellA = cellW * cellH;
            if (cellA <= 0.0) continue;

            const double mult = field[j*nx + i];
            const double lambda_cell = lambda * mult;

            std::poisson_distribution<int> pois(lambda_cell * cellA);
            int Ncell = pois(rng);

            for (int k = 0; k < Ncell; ++k) {
                const double d = P.diam.sample(rng);
                const double r = 0.5 * d;

                // if the nodule can't fit in this cell (or patch), skip it
                if (2*r >= cellW || 2*r >= cellH) continue;

                bool placed = false;
                for (int attempt = 0; attempt < P.max_attempts_per_nodule; ++attempt) {
                    const double x = x0 + r + (cellW - 2*r) * U01(rng);
                    const double y = y0 + r + (cellH - 2*r) * U01(rng);

                    if (ok_no_overlap(x, y, r)) {
                        // build ChBody
                        std::shared_ptr<chrono::ChBody> ball = chrono_types::make_shared<chrono::ChBodyEasySphere>(
                            d / 2.0,     // radius
                            1000.0,   // density
                            true,     // visual
                            true,     // collision
                            sys->GetMat() // mat
                        );

                        out.push_back(Nodule{x, y, d, ball});
                        insert_grid(static_cast<int>(out.size() - 1));
                        placed = true;
                        break;
                    }
                }
                (void)placed; // if not placed, we just drop it
            }
        }
    }

    return out;
}
