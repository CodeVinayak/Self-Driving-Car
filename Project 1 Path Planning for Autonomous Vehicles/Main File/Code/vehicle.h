#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>
#include <vector>

using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, float s, float speed);

  // Destructor
  virtual ~Vehicle();

  void AssessOtherCar(Vehicle car);
  double ChooseNextMove(double ref_vel);
  float CalculateCost(string state);
  double KeepLane(double ref_vela);

  const int MIN_SAFE_DISTANCE = 30;
  const double MAX_SPEED = 49.5;
  const double MAX_ACC = 0.224;

  int lane;
  float s, d, speed, acceleration;
  bool car_ahead, car_right, car_left;
  float car_ahead_speed, car_ahead_distance;
  float car_left_speed, car_left_distance;
  float car_right_speed, car_right_distance;
};

#endif  // VEHICLE_H
