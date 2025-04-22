//
// Created by Nikash Jakkidi on 4/22/25.
//

#include "file_utils.h"
#include <fstream>
#include <sstream>

std::vector<City> readCitiesWithinBox(const std::string& filename, double minLat, double maxLat, double minLon, double maxLon) {
    std::ifstream infile(filename);
    std::vector<City> cities;
    std::string line;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string name, popStr, latStr, lonStr, classType;
        std::getline(ss, name, '\t');
        std::getline(ss, popStr, '\t');
        std::getline(ss, latStr, '\t');
        std::getline(ss, lonStr, '\t');
        std::getline(ss, classType, '\t');

        if (classType != "P") continue;

        try {
            double lat = std::stod(latStr);
            double lon = std::stod(lonStr);
            int pop = std::stoi(popStr);
            if (lat >= minLat && lat <= maxLat && lon >= minLon && lon <= maxLon)
                cities.push_back({name, pop, lat, lon});
        } catch (...) { continue; }
    }
    return cities;
}

std::vector<Landmark> readLandmarksWithinBox(const std::string& filename, double minLat, double maxLat, double minLon, double maxLon) {
    std::ifstream infile(filename);
    std::vector<Landmark> landmarks;
    std::string line;
    while (std::getline(infile, line)) {
        std::stringstream ss(line);
        std::string name, popStr, latStr, lonStr, classType;
        std::getline(ss, name, '\t');
        std::getline(ss, popStr, '\t');
        std::getline(ss, latStr, '\t');
        std::getline(ss, lonStr, '\t');
        std::getline(ss, classType, '\t');

        if (classType != "L") continue;

        try {
            double lat = std::stod(latStr);
            double lon = std::stod(lonStr);
            if (lat >= minLat && lat <= maxLat && lon >= minLon && lon <= maxLon)
                landmarks.push_back({name, lat, lon});
        } catch (...) { continue; }
    }
    return landmarks;
}
