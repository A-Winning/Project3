//
// Created by Nikash Jakkidi on 4/22/25.
//

#pragma once
#include "geo_utils.h"
#include <vector>
#include <queue>

struct KDNode {
    Landmark point;
    KDNode* left = nullptr;
    KDNode* right = nullptr;
    int axis;
};

KDNode* buildKDTree(std::vector<Landmark>& points, int depth = 0);

std::vector<Landmark> findNearestLandmarksKD(KDNode* root, const City& city, int k = 5);

