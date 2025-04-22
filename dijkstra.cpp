//
// Created by Nikash Jakkidi on 4/22/25.
//
#include "dijkstra.h"
#include <limits>
#include <queue>

std::vector<int> dijkstra(int start, int end, const std::vector<std::vector<double>>& graph) {
    int n = graph.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> prev(n, -1);
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    dist[start] = 0;
    pq.emplace(0, start);

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (u == end) break;

        for (int v = 0; v < n; ++v) {
            if (u == v) continue;
            double alt = dist[u] + graph[u][v];
            if (alt < dist[v]) {
                dist[v] = alt;
                prev[v] = u;
                pq.emplace(alt, v);
            }
        }
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = prev[at]) path.push_back(at);
    std::reverse(path.begin(), path.end());
    return path;
}