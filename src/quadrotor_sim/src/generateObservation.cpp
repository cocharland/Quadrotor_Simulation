//
// Created by cody on 1/28/20.
//

#include <random>
#include <iostream>
#include <chrono>
#include "generateObservation.h"

std::vector<int> generate_observation( std::vector<int> probabilities){
    //takes a probability vector and returns a converted observation over the full range for now
    std::vector<int> returned_vals;
    int n = 0;

    unsigned  seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::cout << probabilities.size() << std::endl;
    for (int j = 0; j < probabilities.size(); j++){

        std::binomial_distribution<int> distribution(1,((float)probabilities[j] / 100));
        int result = distribution(generator);
        n += result;
        returned_vals.push_back(result);

        std::cout << n <<" : " << ((float)probabilities[j] / 100) << std::endl;
    }
    return returned_vals;
}