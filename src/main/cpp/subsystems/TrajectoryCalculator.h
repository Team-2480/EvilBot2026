
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>

class TrajectoryCalculator {
 public:
  TrajectoryCalculator() {}
  ~TrajectoryCalculator() {}

  float calculate_point(double x) {
    return std::tan(pitch) * x -
           grav * (std::pow(x, 2) /
                   (2 * std::pow(std::cos(pitch), 2) * std::pow(velocity, 2)));
  }

  void set_fixed(double c_pitch) {
    pitch = c_pitch;
    fixed = true;
  }

  double calculate_loss(double targ_x, double targ_y) {
    if (!fixed) {
      pitch = std::atan(targ_y / (targ_x / 4));
    }

    double loss =
        std::tan(pitch) * targ_x -
        grav * (std::pow(targ_x, 2) /
                (2 * std::pow(std::cos(pitch), 2) * std::pow(velocity, 2))) -
        targ_y;

    if (loss < 0) {
      velocity += std::clamp(std::abs(loss / 500), 0.1, 50.0);
    } else {
      velocity -= std::clamp(std::abs(loss / 500), 0.1, 50.0);
    }
    velocity = std::clamp(velocity, 0.0, max_speed);

    return loss;
  }

 private:
  double pitch = 0;
  double velocity = 1;
  bool fixed = false;

  static constexpr double grav = 32.174;
  static constexpr double max_speed = 50.0;
};
