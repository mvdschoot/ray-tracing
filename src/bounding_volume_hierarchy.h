#pragma once
#include "ray_tracing.h"
#include "scene.h"
#include <array>
#include <gsl-lite/gsl-lite.hpp>

struct Primitive
{
	unsigned int triangle;
	int mesh_idx;
	AxisAlignedBox aabb;
	bool isSphere;

	/*bool operator < (const Primitive& other) const
	{
		float centroid = (aabb.upper.x + aabb.lower.x) / 2.0f;
		float centroid_other = (other.aabb.upper.x + other.aabb.lower.x) / 2.0f;

		return (centroid < centroid_other);
	}*/
};

class BoundingVolumeHierarchy {

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

	std::vector<Node> nodes;

	void build(std::vector<Primitive> primitives, int depth, int idx);
	void countLevels(std::vector<Primitive> primitives, int depth);
	AxisAlignedBox getAABB(std::vector<Primitive> primitives);
	AxisAlignedBox getAABB(Sphere sphere);


    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo, bool interpolate, int idx) const;
	bool intersectPrimitive(Node node, Ray& ray, HitInfo& hitInfo, bool interpolate) const;
private:
	bool AABBIntersect(Ray& ray, const AxisAlignedBox aabb) const;
	void sortPrimitives(std::vector<Primitive>& primitives);
	void SAHsplit(AxisAlignedBox aabb, std::vector<Primitive> primitives);
	float getVolume(AxisAlignedBox aabb);

	Scene* m_pScene;
};

