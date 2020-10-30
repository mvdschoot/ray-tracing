#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <array>
#include <gsl-lite/gsl-lite.hpp>


class BoundingVolumeHierarchy {

	struct Primitive
	{
		Triangle triangle;
		int mesh_idx;
		AxisAlignedBox aabb;

		bool operator < (const Primitive& other) const
		{
			float centroid_x = (aabb.upper.x + aabb.lower.x) / 2.0f;
			float centroid_other_x = (other.aabb.upper.x + other.aabb.lower.x) / 2.0f;
			return (centroid_x < centroid_other_x);
		}
	};

	struct Node
	{
		AxisAlignedBox aabb;
		int depth;
		int idx;

		bool isLeaf = false;
		std::vector<Primitive> primitives;
	};



public:
	BoundingVolumeHierarchy(Scene* pScene, const int MAX_BVH_LEVEL);

	// Use this function to visualize your BVH. This can be useful for debugging.
	void debugDraw(int level);
	int numLevels();
	int levels = std::numeric_limits<float>::min();
	int MAX_BVH_LEVEL;

	// Return true if something is hit, returns false otherwise.
	// Only find hits if they are closer than t stored in the ray and the intersection
	// is on the correct side of the origin (the new t >= 0).
	bool intersect(Ray& ray, HitInfo& hitInfo) const;

	std::vector<Node> nodes;

	void build(std::vector<Primitive> primitives, int depth, int idx);
	void countLevels(std::vector<Primitive> primitives, int depth);
	AxisAlignedBox getAABB(std::vector<Primitive> primitives);


    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo, bool interpolate, int idx) const;
	bool intersectTriangles(Node node, Ray& ray, HitInfo& hitInfo, bool interpolate) const;

private:
	Scene* m_pScene;
	int maxLvlIdx;
};

