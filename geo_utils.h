//
// Created by Nikash Jakkidi on 4/22/25.
//


#pragma once
#include <string>
#include <cmath>

const double EARTH_RADIUS_KM = 6371.0;

struct City {
    std::string name;
    int population;
    double lat;
    double lon;
};

struct Landmark {
    std::string name;
    double lat;
    double lon;
};

double haversine(double lat1, double lon1, double lat2, double lon2);


