#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

class HeposGridCorrector {
public:
    struct HeposGrid {
        int                 rows      = 0;
        int                 cols      = 0;
        double              grid_size = 0.0;
        double              header4   = 0.0;
        double              header5   = 0.0;
        std::vector<double> data; // HEPOS grid values are stored in centimeters.

        bool valid() const {
            return rows > 0 && cols > 0 && grid_size > 0.0 &&
                   data.size() == static_cast<size_t>(rows) * static_cast<size_t>(cols);
        }

        double at(size_t row, size_t col) const { return data[row * static_cast<size_t>(cols) + col]; }
    };

    struct Params {
        std::string e_grid_path;
        std::string n_grid_path;
        double      grid_spacing_m            = 2000.0;
        double      origin_x                  = -64400.0;   // EPSG:2100 grid origin E, meters
        double      origin_y                  = 3840000.0;  // EPSG:2100 grid origin N, meters
        bool        x_increases_east          = true;
        bool        y_increases_north         = true;
        double      local_offset_e            = 0.0;
        double      local_offset_n            = 0.0;
        bool        use_local_offset_fallback = false;
        bool        grid_value_in_cm          = true;
    };

    struct CorrectionResult {
        bool   applied       = false;
        bool   used_grid     = false;
        bool   used_fallback = false;
        double grid_x        = 0.0;
        double grid_y        = 0.0;
        double grid_col      = 0.0;
        double grid_row      = 0.0;
        double delta_e_cm    = 0.0;
        double delta_n_cm    = 0.0;
        double delta_e_m     = 0.0;
        double delta_n_m     = 0.0;
    };

    HeposGridCorrector() = default;

    bool load(const Params &params, const rclcpp::Logger &logger) {
        params_ = params;
        logger_ = logger;
        loaded_ = load_grid(params_.e_grid_path, e_grid_, logger_) && load_grid(params_.n_grid_path, n_grid_, logger_);
        if (!loaded_) {
            RCLCPP_WARN(logger_, "HEPOS grid correction disabled: failed to load grid files.");
            return false;
        }

        if (e_grid_.rows != n_grid_.rows || e_grid_.cols != n_grid_.cols ||
            std::abs(e_grid_.grid_size - n_grid_.grid_size) > 1e-9) {
            RCLCPP_WARN(logger_, "HEPOS grid header mismatch between E/N grid files.");
            loaded_ = false;
            return false;
        }

        if (params_.grid_spacing_m <= 0.0) {
            params_.grid_spacing_m = e_grid_.grid_size;
        }
        if (params_.origin_x == 0.0 && params_.origin_y == 0.0) {
            // This HEPOS dE/dN grid is sampled by EPSG:2100 projected E/N,
            // not by TM07.  The grid origin that matches the validation points is:
            //   E_min = -64400 m, N_min = 3840000 m.
            params_.origin_x = -64400.0;
            params_.origin_y = 3840000.0;
        }

        RCLCPP_INFO(logger_, "HEPOS grid loaded: rows=%d cols=%d spacing=%.3f origin=(%.3f, %.3f)", e_grid_.rows,
                    e_grid_.cols, params_.grid_spacing_m, params_.origin_x, params_.origin_y);
        return true;
    }

    bool enabled() const { return loaded_ || params_.use_local_offset_fallback; }

    bool correct(double &x, double &y) const { return correct(x, y, x, y); }

    bool correct(double &x, double &y, double grid_x, double grid_y) const {
        CorrectionResult result;
        return correct_with_result(x, y, grid_x, grid_y, result);
    }

    bool correct_with_result(double &x, double &y, double grid_x, double grid_y, CorrectionResult &result) const {
        result        = CorrectionResult{};
        result.grid_x = grid_x;
        result.grid_y = grid_y;
        compute_grid_index(grid_x, grid_y, result.grid_col, result.grid_row);

        if (loaded_) {
            double de  = 0.0;
            double dn  = 0.0;
            double col = 0.0;
            double row = 0.0;
            if (sample_grid(grid_x, grid_y, e_grid_, de, col, row) &&
                sample_grid(grid_x, grid_y, n_grid_, dn, col, row)) {
                result.used_grid  = true;
                result.grid_col   = col;
                result.grid_row   = row;
                result.delta_e_cm = de;
                result.delta_n_cm = dn;
                if (params_.grid_value_in_cm) {
                    de *= 0.01;
                    dn *= 0.01;
                }
                result.delta_e_m = de;
                result.delta_n_m = dn;
                x += de;
                y += dn;
                result.applied = true;
                return true;
            }
        }

        if (params_.use_local_offset_fallback) {
            x += params_.local_offset_e;
            y += params_.local_offset_n;
            result.applied       = true;
            result.used_fallback = true;
            result.delta_e_m     = params_.local_offset_e;
            result.delta_n_m     = params_.local_offset_n;
            result.delta_e_cm    = params_.local_offset_e * 100.0;
            result.delta_n_cm    = params_.local_offset_n * 100.0;
            return true;
        }

        return false;
    }

private:
    bool load_grid(const std::string &path, HeposGrid &grid, const rclcpp::Logger &logger) {
        grid = HeposGrid{};

        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger, "Failed to open HEPOS grid file: %s", path.c_str());
            return false;
        }

        if (!(file >> grid.rows >> grid.cols >> grid.grid_size >> grid.header4 >> grid.header5)) {
            RCLCPP_ERROR(logger, "Failed to read HEPOS grid header: %s", path.c_str());
            return false;
        }

        if (grid.rows <= 0 || grid.cols <= 0 || grid.grid_size <= 0.0) {
            RCLCPP_ERROR(logger, "Invalid HEPOS grid header: rows=%d cols=%d grid_size=%.3f", grid.rows, grid.cols,
                         grid.grid_size);
            return false;
        }

        const size_t total = static_cast<size_t>(grid.rows) * static_cast<size_t>(grid.cols);
        grid.data.assign(total, 0.0);
        for (size_t i = 0; i < total; ++i) {
            if (!(file >> grid.data[i])) {
                RCLCPP_ERROR(logger, "HEPOS grid data read error at index %zu: %s", i, path.c_str());
                grid.data.clear();
                return false;
            }
        }

        RCLCPP_INFO(logger, "Loaded HEPOS grid file: %s rows=%d cols=%d grid_size=%.3f header4=%.3f header5=%.3f",
                    path.c_str(), grid.rows, grid.cols, grid.grid_size, grid.header4, grid.header5);
        return true;
    }

    bool sample_grid(double x, double y, const HeposGrid &grid, double &value) const {
        double col = 0.0;
        double row = 0.0;
        return sample_grid(x, y, grid, value, col, row);
    }

    bool sample_grid(double x, double y, const HeposGrid &grid, double &value, double &col, double &row) const {
        if (!grid.valid() || params_.grid_spacing_m <= 0.0) {
            return false;
        }

        compute_grid_index(x, y, col, row);

        if (col < 0.0 || row < 0.0) {
            return false;
        }

        const size_t cols = static_cast<size_t>(grid.cols);
        const size_t rows = static_cast<size_t>(grid.rows);
        if (col > static_cast<double>(cols - 1) || row > static_cast<double>(rows - 1)) {
            return false;
        }

        const size_t c0 = static_cast<size_t>(std::floor(col));
        const size_t r0 = static_cast<size_t>(std::floor(row));
        const size_t c1 = std::min(c0 + 1, cols - 1);
        const size_t r1 = std::min(r0 + 1, rows - 1);
        const double tx = col - static_cast<double>(c0);
        const double ty = row - static_cast<double>(r0);

        const double v00 = grid.at(r0, c0);
        const double v10 = grid.at(r0, c1);
        const double v01 = grid.at(r1, c0);
        const double v11 = grid.at(r1, c1);

        const double v0 = v00 * (1.0 - tx) + v10 * tx;
        const double v1 = v01 * (1.0 - tx) + v11 * tx;
        value           = v0 * (1.0 - ty) + v1 * ty;
        return true;
    }

    void compute_grid_index(double x, double y, double &col, double &row) const {
        col = (x - params_.origin_x) / params_.grid_spacing_m;
        row = (y - params_.origin_y) / params_.grid_spacing_m;
        if (!params_.x_increases_east) {
            col = (params_.origin_x - x) / params_.grid_spacing_m;
        }
        if (!params_.y_increases_north) {
            row = (params_.origin_y - y) / params_.grid_spacing_m;
        }
    }

    Params         params_;
    rclcpp::Logger logger_ = rclcpp::get_logger("hepos_grid_corrector");
    bool           loaded_ = false;
    HeposGrid      e_grid_;
    HeposGrid      n_grid_;
};
