#include "tbai_deploy_g1/PBHCMotionLoader.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace tbai {
namespace g1 {

constexpr int PBHC_NUM_JOINTS = 23;

PBHCMotionLoader::PBHCMotionLoader(const std::string &motionFilePath)
    : numFrames_(0),
      fps_(50.0f),
      dt_(0.02f),
      currentTime_(0.0f),
      timeStart_(0.0f),
      timeEnd_(-1.0f),
      currentRootPos_(vector_t::Zero(3)),
      currentRootRot_(vector_t::Zero(4)),
      currentDofPos_(vector_t::Zero(PBHC_NUM_JOINTS)),
      currentDofVel_(vector_t::Zero(PBHC_NUM_JOINTS)) {
    logger_ = tbai::getLogger("PBHCMotionLoader");

    loadCsvFile(motionFilePath);

    // Set default quat to identity if not set
    currentRootRot_(3) = 1.0;  // w component

    if (timeEnd_ < 0) {
        timeEnd_ = getDuration();
    }

    TBAI_LOG_INFO(logger_, "Loaded motion with {} frames, fps={}, duration={:.2f}s", numFrames_, fps_, getDuration());
}

void PBHCMotionLoader::loadCsvFile(const std::string &motionFilePath) {
    // CSV format for PBHC:
    // First line: fps
    // Per frame: px,py,pz,qx,qy,qz,qw,j0,...,j22,jv0,...,jv22
    // Total: 3 + 4 + 23 + 23 = 53 values per row

    std::ifstream file(motionFilePath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open motion file: " + motionFilePath);
    }

    std::vector<std::vector<double>> data;
    std::string line;

    // First line: fps
    if (std::getline(file, line)) {
        fps_ = std::stof(line);
        dt_ = 1.0f / fps_;
    }

    // Read motion data
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value));
        }

        // Expected: px,py,pz,qx,qy,qz,qw,j0,...,j22,jv0,...,jv22 = 53 values
        // Or minimal: px,py,pz,qx,qy,qz,qw,j0,...,j22 = 30 values (no vel)
        if (row.size() >= 30) {
            data.push_back(row);
        }
    }

    numFrames_ = static_cast<int>(data.size());
    if (numFrames_ == 0) {
        throw std::runtime_error("No valid motion frames loaded");
    }

    // Allocate matrices
    rootPos_.resize(3, numFrames_);
    rootRot_.resize(4, numFrames_);
    dofPos_.resize(PBHC_NUM_JOINTS, numFrames_);
    dofVel_.resize(PBHC_NUM_JOINTS, numFrames_);

    bool hasVelocities = data[0].size() >= 53;

    // Fill matrices
    for (int i = 0; i < numFrames_; ++i) {
        const auto &row = data[i];

        // Root position (0-2)
        rootPos_(0, i) = row[0];
        rootPos_(1, i) = row[1];
        rootPos_(2, i) = row[2];

        // Root quaternion (3-6): x,y,z,w
        rootRot_(0, i) = row[3];
        rootRot_(1, i) = row[4];
        rootRot_(2, i) = row[5];
        rootRot_(3, i) = row[6];

        // Joint positions (7-29)
        for (int j = 0; j < PBHC_NUM_JOINTS; ++j) {
            dofPos_(j, i) = row[7 + j];
        }

        // Joint velocities (30-52) if available
        if (hasVelocities) {
            for (int j = 0; j < PBHC_NUM_JOINTS; ++j) {
                dofVel_(j, i) = row[30 + j];
            }
        } else {
            // Compute velocities from position differences
            if (i > 0) {
                for (int j = 0; j < PBHC_NUM_JOINTS; ++j) {
                    dofVel_(j, i) = (dofPos_(j, i) - dofPos_(j, i - 1)) / dt_;
                }
            } else {
                dofVel_.col(i).setZero();
            }
        }
    }

    // Set time end to full duration
    timeEnd_ = getDuration();
}

void PBHCMotionLoader::interpolateFrame(float time) {
    // Clamp time to valid range
    time = std::max(timeStart_, std::min(time, timeEnd_));

    // Compute frame indices and blend factor
    float phase = (time - timeStart_) / (timeEnd_ - timeStart_);
    phase = std::max(0.0f, std::min(phase, 1.0f));

    float frameFloat = phase * (numFrames_ - 1);
    int frame0 = static_cast<int>(frameFloat);
    int frame1 = std::min(frame0 + 1, numFrames_ - 1);
    float blend = frameFloat - frame0;

    // Interpolate position
    currentRootPos_ = (1.0 - blend) * rootPos_.col(frame0) + blend * rootPos_.col(frame1);

    // Interpolate quaternion (simple lerp + normalize)
    currentRootRot_ = (1.0 - blend) * rootRot_.col(frame0) + blend * rootRot_.col(frame1);
    currentRootRot_.normalize();

    // Interpolate DOF positions
    currentDofPos_ = (1.0 - blend) * dofPos_.col(frame0) + blend * dofPos_.col(frame1);

    // Interpolate DOF velocities
    currentDofVel_ = (1.0 - blend) * dofVel_.col(frame0) + blend * dofVel_.col(frame1);
}

scalar_t PBHCMotionLoader::getRefMotionPhase() const {
    if (timeEnd_ <= timeStart_) {
        return 0.0;
    }
    return static_cast<scalar_t>((currentTime_ - timeStart_) / (timeEnd_ - timeStart_));
}

vector_t PBHCMotionLoader::getDofPositions() const {
    return currentDofPos_;
}

vector_t PBHCMotionLoader::getDofVelocities() const {
    return currentDofVel_;
}

vector_t PBHCMotionLoader::getRootPosition() const {
    return currentRootPos_;
}

vector_t PBHCMotionLoader::getRootQuaternion() const {
    return currentRootRot_;
}

bool PBHCMotionLoader::advance(float dt) {
    currentTime_ += dt;

    if (currentTime_ >= timeEnd_) {
        currentTime_ = timeEnd_;
        interpolateFrame(currentTime_);
        return false;  // Motion complete
    }

    interpolateFrame(currentTime_);
    return true;  // Motion still active
}

void PBHCMotionLoader::reset() {
    currentTime_ = timeStart_;
    interpolateFrame(currentTime_);
}

void PBHCMotionLoader::setTimeRange(float timeStart, float timeEnd) {
    timeStart_ = std::max(0.0f, timeStart);
    timeEnd_ = (timeEnd < 0) ? getDuration() : std::min(timeEnd, getDuration());
    reset();
}

float PBHCMotionLoader::getDuration() const {
    return (numFrames_ - 1) * dt_;
}

}  // namespace g1
}  // namespace tbai
