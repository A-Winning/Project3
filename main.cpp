// main.cpp
#include <iostream>
#include "file_utils.h"
#include "geo_utils.h"
#include "kdtree.h"
#include "dijkstra.h"

int main() {
    double startLat, startLon, endLat, endLon;
    int numCities;

    std::cout << "Enter start latitude and longitude: ";
    std::cin >> startLat >> startLon;
    std::cout << "Enter end latitude and longitude: ";
    std::cin >> endLat >> endLon;
    std::cout << "Enter number of cities to visit: ";
    std::cin >> numCities;

    double minLat = std::min(startLat, endLat);
    double maxLat = std::max(startLat, endLat);
    double minLon = std::min(startLon, endLon);
    double maxLon = std::max(startLon, endLon);

    std::vector<City> candidates = readCitiesWithinBox("US_trimmed.txt", minLat, maxLat, minLon, maxLon);
    std::sort(candidates.begin(), candidates.end(), [](const City& a, const City& b) {
        return a.population > b.population;
    });
    if (candidates.size() > 100) candidates.resize(100);

    std::vector<Landmark> landmarks = readLandmarksWithinBox("US_trimmed.txt", minLat, maxLat, minLon, maxLon);
    KDNode* landmarkTree = buildKDTree(landmarks);

    int N = candidates.size();
    std::vector<std::vector<double>> graph(N, std::vector<double>(N));
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            graph[i][j] = haversine(candidates[i].lat, candidates[i].lon, candidates[j].lat, candidates[j].lon);

    double midLat = (startLat + endLat) / 2.0;
    double midLon = (startLon + endLon) / 2.0;
    std::vector<std::pair<double, int>> distToMid;
    for (int i = 0; i < N; ++i) {
        double d = haversine(midLat, midLon, candidates[i].lat, candidates[i].lon);
        distToMid.emplace_back(d, i);
    }
    std::sort(distToMid.begin(), distToMid.end());

    std::vector<int> route;
    for (int i = 0; i < numCities && i < distToMid.size(); ++i)
        route.push_back(distToMid[i].second);

    int closestToStart = -1, closestToEnd = -1;
    double minStartDist = 1e9, minEndDist = 1e9;
    for (int i = 0; i < N; ++i) {
        double d1 = haversine(startLat, startLon, candidates[i].lat, candidates[i].lon);
        double d2 = haversine(endLat, endLon, candidates[i].lat, candidates[i].lon);
        if (d1 < minStartDist) { minStartDist = d1; closestToStart = i; }
        if (d2 < minEndDist) { minEndDist = d2; closestToEnd = i; }
    }

    if (closestToStart != -1) route.insert(route.begin(), closestToStart);
    if (closestToEnd != -1) route.push_back(closestToEnd);

    std::cout << "\nVisiting cities:\n";
    for (int idx : route) {
        const City& city = candidates[idx];
        std::cout << " - " << city.name << " (Pop: " << city.population << ", " << city.lat << ", " << city.lon << ")\n";
        std::vector<Landmark> nearby = findNearestLandmarksKD(landmarkTree, city, 5);
        for (const auto& lm : nearby)
            std::cout << "    -> Landmark: " << lm.name << " (" << lm.lat << ", " << lm.lon << ")\n";
    }

    std::cout << "\nOptimized path between stops:\n";
    for (size_t i = 0; i + 1 < route.size(); ++i) {
        std::vector<int> path = dijkstra(route[i], route[i + 1], graph);
        for (size_t j = 0; j < path.size(); ++j) {
            std::cout << candidates[path[j]].name;
            if (j + 1 < path.size()) std::cout << " -> ";
        }
        std::cout << "\n";
    }

    return 0;
}
