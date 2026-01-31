#include "tbai_deploy_g1/Twist2MotionLoader.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace tbai {
namespace g1 {

Twist2MotionLoader::Twist2MotionLoader(const std::string &motionFilePath)
    : numFrames_(0),
      fps_(50.0f),
      dt_(0.02f),
      currentTime_(0.0f),
      timeStart_(0.0f),
      timeEnd_(-1.0f),
      currentRootPos_(vector_t::Zero(3)),
      currentRootRot_(vector_t::Zero(4)),
      currentRootVel_(vector_t::Zero(3)),
      currentRootAngVel_(vector_t::Zero(3)),
      currentDofPos_(vector_t::Zero(29)) {
    logger_ = tbai::getLogger("Twist2MotionLoader");

    auto ends_with = [](const std::string &str, const std::string &suffix) {
        return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
    };

    if (!ends_with(motionFilePath, ".csv")) {
        throw std::runtime_error("Twist2MotionLoader: Only CSV format supported");
    }

    loadCsvFile(motionFilePath);

    // Set default quat to identity if not set
    currentRootRot_(3) = 1.0;  // w component

    if (timeEnd_ < 0) {
        timeEnd_ = getDuration();
    }

    TBAI_LOG_INFO(logger_, "Loaded motion with {} frames, fps={}, duration={:.2f}s", numFrames_, fps_, getDuration());
}

void Twist2MotionLoader::loadCsvFile(const std::string &motionFilePath) {
    std::ifstream file(motionFilePath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open motion file: " + motionFilePath);
    }

    std::vector<std::vector<double>> data;
    std::string line;

    if (std::getline(file, line)) {
        fps_ = std::stof(line);
        dt_ = 1.0f / fps_;
    }

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;

        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value));
        }

        // Expected: px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,j0,...,j28 = 42 values
        if (row.size() >= 42) {
            data.push_back(row);
        }
    }

    numFrames_ = static_cast<int>(data.size());
    if (numFrames_ == 0) {
        throw std::runtime_error("No valid motion frames loaded");
    }

    rootPos_.resize(3, numFrames_);
    rootRot_.resize(4, numFrames_);
    rootVel_.resize(3, numFrames_);
    rootAngVel_.resize(3, numFrames_);
    dofPos_.resize(29, numFrames_);

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

        // Root linear velocity (7-9)
        rootVel_(0, i) = row[7];
        rootVel_(1, i) = row[8];
        rootVel_(2, i) = row[9];

        // Root angular velocity (10-12)
        rootAngVel_(0, i) = row[10];
        rootAngVel_(1, i) = row[11];
        rootAngVel_(2, i) = row[12];

        // Joint positions (13-41)
        for (int j = 0; j < 29; ++j) {
            dofPos_(j, i) = row[13 + j];
        }
    }

    timeEnd_ = getDuration();
}

void Twist2MotionLoader::interpolateFrame(float time) {
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

    // Interpolate quaternion (simple lerp + normalize for small angles)
    currentRootRot_ = (1.0 - blend) * rootRot_.col(frame0) + blend * rootRot_.col(frame1);
    currentRootRot_.normalize();

    // Velocities from frame0 (no interpolation needed for velocities)
    currentRootVel_ = rootVel_.col(frame0);
    currentRootAngVel_ = rootAngVel_.col(frame0);

    // Interpolate DOF positions
    currentDofPos_ = (1.0 - blend) * dofPos_.col(frame0) + blend * dofPos_.col(frame1);
}

vector_t Twist2MotionLoader::rotateToLocal(const vector_t &vec, const vector_t &quat) const {
    double qx = quat(0);
    double qy = quat(1);
    double qz = quat(2);
    double qw = quat(3);

    double vx = vec(0);
    double vy = vec(1);
    double vz = vec(2);

    // a = v * (2*w^2 - 1)
    double scale_a = 2.0 * qw * qw - 1.0;
    double ax = vx * scale_a;
    double ay = vy * scale_a;
    double az = vz * scale_a;

    // b = cross(q_vec, v) * 2*w
    double bx = (qy * vz - qz * vy) * 2.0 * qw;
    double by = (qz * vx - qx * vz) * 2.0 * qw;
    double bz = (qx * vy - qy * vx) * 2.0 * qw;

    // c = q_vec * 2*dot(q_vec, v)
    double dot = qx * vx + qy * vy + qz * vz;
    double cx = qx * 2.0 * dot;
    double cy = qy * 2.0 * dot;
    double cz = qz * 2.0 * dot;

    vector_t result(3);
    result(0) = ax - bx + cx;
    result(1) = ay - by + cy;
    result(2) = az - bz + cz;

    return result;
}

vector_t Twist2MotionLoader::quaternionToEuler(const vector_t &quat) const {
    // Convert quaternion (x,y,z,w) to euler angles (roll, pitch, yaw)
    double x = quat(0);
    double y = quat(1);
    double z = quat(2);
    double w = quat(3);

    vector_t euler(3);

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler(0) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        euler(1) = std::copysign(M_PI / 2.0, sinp);
    } else {
        euler(1) = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler(2) = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}

vector_t Twist2MotionLoader::getActionMimic() const {
    // Build 35-dim action_mimic:
    // xy_vel(2) + z(1) + roll(1) + pitch(1) + yaw_ang_vel(1) + dof_pos(29)

    vector_t actionMimic(35);

    // Get velocities in local frame
    vector_t localVel = rotateToLocal(currentRootVel_, currentRootRot_);
    vector_t localAngVel = rotateToLocal(currentRootAngVel_, currentRootRot_);

    // xy velocity (local frame)
    actionMimic(0) = localVel(0);
    actionMimic(1) = localVel(1);

    // z position
    actionMimic(2) = currentRootPos_(2);

    // Roll and pitch from quaternion
    vector_t euler = quaternionToEuler(currentRootRot_);
    actionMimic(3) = euler(0);  // roll
    actionMimic(4) = euler(1);  // pitch

    // Yaw angular velocity (local frame)
    actionMimic(5) = localAngVel(2);

    // 29 DOF positions
    actionMimic.segment(6, 29) = currentDofPos_;

    return actionMimic;
}

vector_t Twist2MotionLoader::getRootPosition() const {
    return currentRootPos_;
}

vector_t Twist2MotionLoader::getRootQuaternion() const {
    return currentRootRot_;
}

vector_t Twist2MotionLoader::getRootVelocity() const {
    return currentRootVel_;
}

vector_t Twist2MotionLoader::getRootAngularVelocity() const {
    return currentRootAngVel_;
}

vector_t Twist2MotionLoader::getDofPositions() const {
    return currentDofPos_;
}

bool Twist2MotionLoader::advance(float dt) {
    currentTime_ += dt;

    if (currentTime_ >= timeEnd_) {
        currentTime_ = timeEnd_;
        interpolateFrame(currentTime_);
        return false;  // Motion complete
    }

    interpolateFrame(currentTime_);
    return true;  // Motion still active
}

void Twist2MotionLoader::reset() {
    currentTime_ = timeStart_;
    interpolateFrame(currentTime_);
}

void Twist2MotionLoader::setTimeRange(float timeStart, float timeEnd) {
    timeStart_ = std::max(0.0f, timeStart);
    timeEnd_ = (timeEnd < 0) ? getDuration() : std::min(timeEnd, getDuration());
    reset();
}

float Twist2MotionLoader::getDuration() const {
    return (numFrames_ - 1) * dt_;
}

}  // namespace g1
}  // namespace tbai
