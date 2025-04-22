//
// Created by Nikash Jakkidi on 4/22/25.
//
#include "kdtree.h"
#include <algorithm>

KDNode* buildKDTree(std::vector<Landmark>& points, int depth) {
    if (points.empty()) return nullptr;

    int axis = depth % 2;
    std::sort(points.begin(), points.end(), [axis](const Landmark& a, const Landmark& b) {
        return axis == 0 ? a.lat < b.lat : a.lon < b.lon;
    });

    int mid = points.size() / 2;
    KDNode* node = new KDNode{points[mid], nullptr, nullptr, axis};

    std::vector<Landmark> leftPoints(points.begin(), points.begin() + mid);
    std::vector<Landmark> rightPoints(points.begin() + mid + 1, points.end());

    node->left = buildKDTree(leftPoints, depth + 1);
    node->right = buildKDTree(rightPoints, depth + 1);

    return node;
}

template<typename Comp>
void kNearest(KDNode* root, const City& target, int k,
              std::priority_queue<std::pair<double, Landmark>,
              std::vector<std::pair<double, Landmark>>, Comp>& best) {
    if (!root) return;

    double dist = haversine(target.lat, target.lon, root->point.lat, root->point.lon);
    if ((int)best.size() < k) best.push({dist, root->point});
    else if (dist < best.top().first) {
        best.pop();
        best.push({dist, root->point});
    }

    int axis = root->axis;
    double tgtVal = axis == 0 ? target.lat : target.lon;
    double nodeVal = axis == 0 ? root->point.lat : root->point.lon;

    KDNode* near = tgtVal < nodeVal ? root->left : root->right;
    KDNode* far = tgtVal < nodeVal ? root->right : root->left;

    kNearest(near, target, k, best);
    if ((int)best.size() < k || std::abs(tgtVal - nodeVal) < best.top().first)
        kNearest(far, target, k, best);
}

std::vector<Landmark> findNearestLandmarksKD(KDNode* root, const City& city, int k) {
    auto comp = [](std::pair<double, Landmark> a, std::pair<double, Landmark> b) {
        return a.first < b.first;
    };

    std::priority_queue<std::pair<double, Landmark>, std::vector<std::pair<double, Landmark>>, decltype(comp)> best(comp);

    kNearest(root, city, k, best);

    std::vector<Landmark> result;
    while (!best.empty()) {
        result.push_back(best.top().second);
        best.pop();
    }
    std::reverse(result.begin(), result.end());
    return result;
}
