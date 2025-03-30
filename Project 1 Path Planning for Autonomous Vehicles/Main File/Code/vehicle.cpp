#include "vehicle.h"
#include <map>
#include <string>
#include <vector>

#include <iostream>
using namespace std;
using std::string;
using std::vector;
using std::min;


// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float speed) {
  this->lane = lane;
  this->s = s;
  this->speed = speed;
  this->acceleration = 0;
  this->car_ahead = false;
  this->car_left = false;
  this->car_right = false;
  this->car_ahead_speed = 0;
  this->car_ahead_distance = 10000;
  this->car_left_speed = 0;
  this->car_left_distance = 10000;
  this->car_right_speed = 0;
  this->car_right_distance = 10000;
}

Vehicle::~Vehicle() {}

void Vehicle::AssessOtherCar(Vehicle car) {
  float distance = car.s - this->s;

  if (car.lane == this->lane) {
    // check if there is a car ahead
    this->car_ahead |= (car.s > this->s) && (car.s - this->s < car.MIN_SAFE_DISTANCE);
    // record the distnace and speed of the car ahead in this lane
    if ((distance < this->car_ahead_distance) && (distance > 0)) {
      this->car_ahead_speed = car.speed;
      this->car_ahead_distance = distance;
    }
  } else if (car.lane == this->lane - 1){
    // check if there is a car in the left lane
    this->car_left |= (distance < car.MIN_SAFE_DISTANCE) && (distance > -car.MIN_SAFE_DISTANCE);
    // record the distnace and speed of the car in the left lane
    if ((distance < this->car_left_distance) && (distance > 0)) {
      this->car_left_speed = car.speed;
      this->car_left_distance = distance;
    }
  } else if (car.lane = this->lane + 1) {
    // check if there is a car in the right lane
    this->car_right |= (distance < car.MIN_SAFE_DISTANCE) && (distance > -car.MIN_SAFE_DISTANCE);
    // record the distnace and speed of the car in the right lane
    if ((distance < this->car_right_distance) && (distance > 0)) {
      this->car_right_speed = car.speed;
      this->car_right_distance = distance;
    }
  }
}

double Vehicle::ChooseNextMove(double ref_vel) {

  // calculate the cost of three possible moves:
  //    1. Change Lane Left (CLL)
  //    2. Change Lane Right (CLR)
  //    3. Keep Lane (KL)
  float cost_left = this->CalculateCost("CLL");
  float cost_right = this->CalculateCost("CLR");
  float cost_keep = this->CalculateCost("KL");

  float cost_min = min(cost_left, min(cost_right,cost_keep));

  // path planning based on mimizing cost
  if (cost_keep == cost_min) {
    ref_vel = this->KeepLane(ref_vel);
  } else if (cost_right == cost_min) {
    this->lane++;
  } else {
    this->lane--;
  }
  return ref_vel;
}


double Vehicle::KeepLane(double ref_vel) {
  if (this->car_ahead) {
    // slow down if there is a car ahead
    this->acceleration = -this->MAX_ACC;
  } else {
    if (ref_vel < this->MAX_SPEED) {
      // speed up until speed reaches to max, there is no car ahead
      this->acceleration = this->MAX_ACC;
    }
  }
  ref_vel += this->acceleration;

  if (ref_vel > this->MAX_SPEED) {
    ref_vel = this->MAX_SPEED;
  }
  return ref_vel;
}


float Vehicle::CalculateCost(string state) {
  float lane_speed = 0;

  if (state == "CLL") {
    if ((this->lane == 0) || this->car_left || !this->car_ahead) {
      return 1;
    } else {
      lane_speed = this->car_left_speed;
    }
  }

  if (state == "CLR") {
    if ((this->lane == 2) || this->car_right || !this->car_ahead) {
      return 1;
    } else {
      lane_speed = this->car_right_speed;
    }
  }

  if (state == "KL") {
    if (car_ahead) return 1;
  } else {
    lane_speed = this->car_ahead_speed;
  }

  // calculate cost function based on speed of the lane, cost is between 0 and 1
  float cost = (this->MAX_SPEED - lane_speed)/this->MAX_SPEED;
  return cost;

}
