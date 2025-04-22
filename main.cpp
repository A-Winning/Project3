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
    std::cout << locations.size() << std::endl;
    for (int i = 0; i < locations.size(); ++i) {
        double d = haversine(target, locations[i]);
        if (d < minDist) {
            minDist = d;
            nearest = i;
        }
    }
    return nearest;
}

// Build graph by connecting cities within a threshold distance
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

// Dijkstra's algorithm for shortest path
std::vector<int> dijkstra(int start, int end,
    const std::unordered_map<int, std::vector<std::pair<int, double>>>& graph) {

    std::vector<double> dist(graph.size(), std::numeric_limits<double>::infinity());
    std::vector<int> prev(graph.size(), -1);
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

int main() {
    std::ifstream file("US_trimmed.txt");
    if (!file.is_open()) {
        std::cerr << "Failed to open US_trimmed.txt\n";
        return 1;
    }

    std::string line;
    std::vector<Location> locations;

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string word;
        std::string name;
        double lat = 0.0, lon = 0.0;
        std::string featureClass;

        // Read name (everything before the first number)
        while (ss >> word) {
            try {
                lat = std::stod(word);  // will throw if not a number
                break;                  // found the latitude
            } catch (...) {
                if (!name.empty()) name += " ";
                name += word;
            }
        }

        // Now read the rest
        if (!(ss >> lon >> featureClass)) {
            //std::cerr << "Malformed line: " << line << "\n";
            continue;
        }

        // Optional: only keep places of type "P" or "L"
        if (featureClass == "P" || featureClass == "L") {
            locations.push_back({name, lat, lon});
        }
    }

    file.close();
    std::cout << "Loaded " << locations.size() << " locations.\n";

    double startLat, startLon, endLat, endLon;
    char comma;  // to consume the commas

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

    std::cout << "Finding route between:\n";
    std::cout << "Start: " << locations[startIdx].name << " (" << locations[startIdx].latitude << ", " << locations[startIdx].longitude << ")\n";
    std::cout << "End:   " << locations[endIdx].name << " (" << locations[endIdx].latitude << ", " << locations[endIdx].longitude << ")\n";

    auto graph = buildGraph(locations);
    auto path = dijkstra(startIdx, endIdx, graph);

    if (path.size() <= 1) {
        std::cout << "No path found between the selected locations.\n";
        return 0;
    }

    std::cout << "\nShortest route:\n";

    for (int idx : path) {
        std::cout << " → " << locations[idx].name << " (" << locations[idx].latitude << ", " << locations[idx].longitude << ")\n";
    }

    return 0;
}


