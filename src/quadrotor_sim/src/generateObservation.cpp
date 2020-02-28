//
// Created by cody on 1/28/20.
//

#include <random>
#include <iostream>
#include <chrono>
#include <math.h>
#include <fstream>
#include "generateObservation.h"

std::vector<int8_t> generate_observation(std::vector<int8_t> probabilities){
    //takes a probability vector and returns a converted observation over the full range for now
    std::vector<int8_t> returned_vals;
    int n = 0;

    unsigned  seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    //std::cout << probabilities.size() << std::endl;
    for (int j = 0; j < probabilities.size(); j++){
        float prob = ((float)probabilities[j] / 100);
        if (prob <= 0){
            prob = .350;
        }
        std::binomial_distribution<int> distribution(1,prob);
        int result = distribution(generator);
        n += result;
        returned_vals.push_back(result);

        //std::cout << n <<" : " << ((float)probabilities[j] / 100) << std::endl;
    }
    return returned_vals;
}
std::vector<int8_t> rayCast(std::vector<int8_t> beliefMap, float theta, int mapX, int mapY, int map_limit_x, int map_limit_y,double map_resolution){
    //need to return the area of the map that may be seen from the vantage point
    //Pack the array
    std::vector<int8_t> observation = generate_observation(beliefMap);
    float temp_map[map_limit_y][map_limit_x];
    float collated_observation[map_limit_y][map_limit_x];
    int8_t resulting_obs[map_limit_y][map_limit_x];
    int row = 0;
    int col = 0;
    for (int j = 0; j < beliefMap.size(); j++){
        temp_map[row][col] = beliefMap[j];
        collated_observation[row][col] = observation.at(j);
        resulting_obs[row][col] = -1;
        if (col == map_limit_x-1){
            row++;
            col = 0;
        } else {
            col++;
        }
    }
    double angle_of_view = 80*M_PI/180; //80 deg2rad
    int num_rays = 60;
    angle_of_view = angle_of_view/2;
    double start_angle = (double) theta - angle_of_view;
    double angle_change = angle_of_view * 2 / num_rays;
    double current_angle = start_angle;
    for (int n = 0; n < num_rays; n++){
        current_angle += angle_change;
        //Now crawl along the ray
        double dist = 0.05; //in cells not meters
        double prevX = mapX;
        double  prevY = mapY;
        double totalDist = 0;
        bool done = false;
        while (totalDist < 15 && !done){
            //Crawl until at 5m limit or into wall/out of map
            totalDist += dist;
            prevX += dist*cos(current_angle);
            prevY += dist*sin(current_angle);
            int tmpX = round(prevX);
            int tmpY = round(prevY);
            //check boundary constraints
            if (tmpX < 0 || tmpX > map_limit_x || tmpY < 0 || tmpY > map_limit_y){
                done = true;
                break;
            }
            if (collated_observation[tmpY][tmpX] == 1){
                //hit a wall on the ray
                resulting_obs[tmpY][tmpX] = 1;
                done = true;
            } else if(collated_observation[tmpY][tmpX] == 0){
                resulting_obs[tmpY][tmpX] = 0;
                //std::cout << tmpX << " " << tmpY << std::endl;
            }
        }
    }
    // For testing, writes the map to a file for matlab to look at.
    std::ofstream testOut;
    testOut.open("Test.txt");
    //std::cout << "MapLimX " << map_limit_x << std::endl;
    for (int row = 0; row < map_limit_y;row++){
        for (int col = 0; col < map_limit_x;col++){
            testOut << std::to_string(resulting_obs[row][col]);
            if (resulting_obs[row][col]==0){
                //std::cout << row << " c: " << col << std::endl;
            }
            if (col < map_limit_x-1){
                testOut << ',';
            }
            //std::cout << std::to_string(resulting_obs[row][col]) << std::endl;
        }
        testOut << '\n';
    }
    testOut.close();
    std::ofstream testOutC;
    testOutC.open("TestCol.txt");
    for (int row = 0; row < map_limit_y;row++){
        for (int col = 0; col < map_limit_x;col++){
            testOutC << std::to_string(collated_observation[row][col]);
            if (col < map_limit_x-1){
                testOutC << ',';
            }
        }
        testOutC << '\n';
    }
    testOutC.close();

    std::vector<int8_t > output;
    row = 0;
    col = 0;
    for (int i = 0; i < map_limit_x*map_limit_y;i++){
        //pack the array
        output.push_back(resulting_obs[row][col]);
        col++;
        if ( col >= map_limit_x){
            col = 0;
            row++;
        }
    }
    return output;
}
std::vector<int8_t> simulate_observation_from_truth(std::vector<int8_t> map, float theta, int mapX, int mapY, int map_limit_x, int map_limit_y){
    std::vector<int8_t> prob_map;
    for (int j = 0; j< map.size(); j++){
        if (map.at(j) == 1){
            prob_map.push_back(97);
        } else {
            prob_map.push_back(3);
        }
    }
    return rayCast(prob_map,theta,mapX,mapY,map_limit_x,map_limit_y,0.5);
}