#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <queue>
#include <limits>
#include <algorithm>

const double EARTH_RADIUS_KM = 6371.0;

struct Location {
    std::string name;
    double latitude;
    double longitude;
};

std::vector<Location> locations;

// Haversine distance between two locations
double haversine(const Location& a, const Location& b) {
    double lat1 = a.latitude * M_PI / 180.0;
    double lon1 = a.longitude * M_PI / 180.0;
    double lat2 = b.latitude * M_PI / 180.0;
    double lon2 = b.longitude * M_PI / 180.0;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double hav = pow(sin(dlat / 2), 2) +
                 cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    return 2 * EARTH_RADIUS_KM * asin(sqrt(hav));
}

// Find the closest location to a coordinate
int findNearestNode(double lat, double lon, const std::vector<Location>& locations) {
    int nearest = -1;
    double minDist = 1e9;
    Location target{"", lat, lon};

    for (int i = 0; i < locations.size(); ++i) {
        double d = haversine(target, locations[i]);
        if (d < minDist) {
            minDist = d;
            nearest = i;
        }
    }
    return nearest;
}

// Build graph by connecting nearby cities
std::unordered_map<int, std::vector<std::pair<int, double>>> buildGraph(
    const std::vector<Location>& locations, double maxDistance = 100.0) {

    std::unordered_map<int, std::vector<std::pair<int, double>>> graph;

    for (int i = 0; i < locations.size(); ++i) {
        for (int j = i + 1; j < locations.size(); ++j) {
            double dist = haversine(locations[i], locations[j]);
            if (dist <= maxDistance) {
                graph[i].emplace_back(j, dist);
                graph[j].emplace_back(i, dist);
            }
        }
    }
    return graph;
}

// Dijkstra's algorithm
std::vector<int> dijkstra(int start, int end,
    const std::unordered_map<int, std::vector<std::pair<int, double>>>& graph) {

    std::vector<double> dist(locations.size(), std::numeric_limits<double>::infinity());
    std::vector<int> prev(locations.size(), -1);
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    dist[start] = 0;
    pq.emplace(0, start);

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (u == end) break;

        for (const auto& [v, weight] : graph.at(u)) {
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                pq.emplace(dist[v], v);
            }
        }
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = prev[at])
        path.push_back(at);

    std::reverse(path.begin(), path.end());
    return path;
}

// Greedy path through selected waypoints
std::vector<int> greedyPath(int startIdx, int endIdx, const std::vector<int>& waypoints,
                            const std::unordered_map<int, std::vector<std::pair<int, double>>>& graph) {
    std::vector<int> path = {startIdx};
    int current = startIdx;
    std::vector<int> remaining = waypoints;

    while (!remaining.empty()) {
        int next = -1;
        double bestDist = std::numeric_limits<double>::infinity();

        for (int w : remaining) {
            auto subPath = dijkstra(current, w, graph);
            if (!subPath.empty()) {
                double dist = 0;
                for (size_t i = 1; i < subPath.size(); ++i)
                    dist += haversine(locations[subPath[i - 1]], locations[subPath[i]]);
                if (dist < bestDist) {
                    bestDist = dist;
                    next = w;
                }
            }
        }

        if (next == -1) break;

        auto segment = dijkstra(current, next, graph);
        if (segment.size() > 1) {
            path.insert(path.end(), segment.begin() + 1, segment.end());
        }

        current = next;
        remaining.erase(std::remove(remaining.begin(), remaining.end(), next), remaining.end());
    }

    auto lastSegment = dijkstra(current, endIdx, graph);
    if (lastSegment.size() > 1) {
        path.insert(path.end(), lastSegment.begin() + 1, lastSegment.end());
    }

    return path;
}

int main() {
    std::ifstream file("US_trimmed.txt");
    if (!file.is_open()) {
        std::cerr << "Failed to open US_trimmed.txt\n";
        return 1;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string word, name;
        double lat = 0.0, lon = 0.0;
        std::string featureClass;

        while (ss >> word) {
            try {
                lat = std::stod(word);
                break;
            } catch (...) {
                if (!name.empty()) name += " ";
                name += word;
            }
        }

        if (!(ss >> lon >> featureClass)) continue;
        if (featureClass == "P" || featureClass == "L") {
            locations.push_back({name, lat, lon});
        }
    }

    file.close();
    std::cout << "Loaded " << locations.size() << " locations.\n";

    double startLat, startLon, endLat, endLon;
    char comma;

    std::cout << "Enter start latitude and longitude (e.g. 25.9, -80.3): ";
    std::cin >> startLat >> comma >> startLon;

    std::cout << "Enter end latitude and longitude (e.g. 29.6, -82.3): ";
    std::cin >> endLat >> comma >> endLon;

    int startIdx = findNearestNode(startLat, startLon, locations);
    int endIdx = findNearestNode(endLat, endLon, locations);

    if (startIdx == -1 || endIdx == -1) {
        std::cerr << "Couldn't find valid nodes near your coordinates.\n";
        return 1;
    }

    int numWaypoints;
    std::cout << "How many landmarks/cities would you like to stop at along the way? ";
    std::cin >> numWaypoints;

    // Pick closest waypoints to midpoint
    Location midpoint = {
        "Midpoint",
        (locations[startIdx].latitude + locations[endIdx].latitude) / 2,
        (locations[startIdx].longitude + locations[endIdx].longitude) / 2
    };

    std::vector<std::pair<double, int>> distances;
    for (int i = 0; i < locations.size(); ++i) {
        if (i == startIdx || i == endIdx) continue;
        double d = haversine(midpoint, locations[i]);
        distances.emplace_back(d, i);
    }
    std::sort(distances.begin(), distances.end());

    std::vector<int> waypointIndices;
    for (int i = 0; i < numWaypoints && i < distances.size(); ++i) {
        waypointIndices.push_back(distances[i].second);
    }

    auto graph = buildGraph(locations, 500.0); // 500km threshold
    auto path = greedyPath(startIdx, endIdx, waypointIndices, graph);

    if (path.size() <= 1) {
        std::cout << "No path found between the selected locations.\n";
        return 0;
    }

    std::cout << "\nShortest route with " << numWaypoints << " stops:\n";
    for (int idx : path) {
        std::cout << " → " << locations[idx].name << " (" << locations[idx].latitude << ", " << locations[idx].longitude << ")\n";
    }

    return 0;
}
