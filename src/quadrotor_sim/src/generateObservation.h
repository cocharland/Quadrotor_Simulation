//
// Created by cody on 1/28/20.
//

#include <vector>

#ifndef QUADROTOR_SIM_GENERATEOBSERVATION_H
#define QUADROTOR_SIM_GENERATEOBSERVATION_H

#endif //QUADROTOR_SIM_GENERATEOBSERVATION_H

std::vector<int8_t> generate_observation(std::vector<int8_t > );
std::vector<int8_t> rayCast(std::vector<int8_t>, float theta, int mapX, int mapY, int, int, double);
std::vector<int8_t> simulate_observation_from_truth(std::vector<int8_t> map, float theta, int mapX, int mapY, int map_limit_x, int map_limit_y);