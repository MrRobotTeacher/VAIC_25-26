#include "DistanceSensor.hpp"

#include "Pose.hpp"
#include "pros/distance.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

DistanceSensor::DistanceSensor(std::uint8_t port, Pose offset, float scaleFactor)
    : pros::Distance { port }
    , offset { offset }
    , scaleFactor { scaleFactor } { };

Pose DistanceSensor::getRay(float heading)
{
    return getRayLocal().rotate(-heading);
}

void DistanceSensor::update()
{
    int dist = get();
    if (dist >= 9999) {
        detectedWall = false;
        return;
    }
    detectedWall = true;
    measured = (float)dist * MM_TO_IN * scaleFactor;
    stddev = 0.2 * measured / std::sqrt(get_confidence() / 64.0);
}

[[nodiscard]] bool DistanceSensor::isValid() const
{
    return detectedWall;
}

[[nodiscard]] float DistanceSensor::prob(Pose particle) const
{
    // .rotate is CCW while particle.theta is CW
    Pose position = particle + offset.rotate(-particle.theta);
    float angle = particle.theta + offset.theta;

    float predicted = std::numeric_limits<float>::max();
    float theta = 0;

    theta = std::abs(std::remainder(angle - 0, TWO_PI));
    if (theta < PI_TWO) {
        predicted = std::min(predicted, (72 - position.y) / std::cos(theta));
    }
    theta = std::abs(std::remainder(angle - PI_TWO, TWO_PI));
    if (theta < PI_TWO) {
        predicted = std::min(predicted, (72 - position.x) / std::cos(theta));
    }
    theta = std::abs(std::remainder(angle - 2 * PI_TWO, TWO_PI));
    if (theta < PI_TWO) {
        predicted = std::min(predicted, (position.y - -72) / std::cos(theta));
    }
    theta = std::abs(std::remainder(angle - 3 * PI_TWO, TWO_PI));
    if (theta < PI_TWO) {
        predicted = std::min(predicted, (position.x - -72) / std::cos(theta));
    }

    // std::cout << "Measured: " << measured << ", predicted: " << predicted << ", stddev: " << stddev << "\n";

    return normalPDF(measured, predicted, stddev);
}

Pose DistanceSensor::getRayLocal()
{
    float dist = (float)get() * MM_TO_IN * scaleFactor;
    Pose sensorRay { 0, dist };
    // offset.theta is compass angle
    sensorRay = sensorRay.rotate(-offset.theta);
    return offset + sensorRay;
}
