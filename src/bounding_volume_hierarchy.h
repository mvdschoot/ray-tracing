#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <array>
#include <gsl-lite/gsl-lite.hpp>


class BoundingVolumeHierarchy {

struct Node
{
    std::vector<int> children;
    AxisAlignedBox aabb;
    bool isLeaf = false;
    int depth;

    bool operator < (const Node& other) const
    {
        float centroid_x = (aabb.upper.x + aabb.lower.x) / 2.0f;
        float centroid_other_x = (other.aabb.upper.x + other.aabb.lower.x) / 2.0f;
        return (centroid_x < centroid_other_x);
    }
};

public:
    BoundingVolumeHierarchy(Scene* pScene);

    // Use this function to visualize your BVH. This can be useful for debugging.
    void debugDraw(int level);
    int numLevels();
    int levels = std::numeric_limits<float>::min();

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo) const;

    std::vector<Node> nodes;

    void build(std::vector<Node> children, int depth);
    AxisAlignedBox getAABB(std::vector<Node> nodes);

    

private:
    Scene* m_pScene;
};

