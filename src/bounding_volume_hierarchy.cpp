#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
#include <iostream>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
	: m_pScene(pScene)
{
	std::vector<Node> leaves;

	for (const auto& mesh : m_pScene->meshes) {
		for (const auto& tri : mesh.triangles) {
			const auto v0 = mesh.vertices[tri[0]];
			const auto v1 = mesh.vertices[tri[1]];
			const auto v2 = mesh.vertices[tri[2]];

			float x_min = glm::min(v0.p.x, glm::min(v1.p.x, v2.p.x));
			float y_min = glm::min(v0.p.y, glm::min(v1.p.y, v2.p.y));
			float z_min = glm::min(v0.p.z, glm::min(v1.p.z, v2.p.z));

			float x_max = glm::max(v0.p.x, glm::max(v1.p.x, v2.p.x));
			float y_max = glm::max(v0.p.y, glm::max(v1.p.y, v2.p.y));
			float z_max = glm::max(v0.p.z, glm::max(v1.p.z, v2.p.z));

			Node node;
			node.aabb = { glm::vec3(x_min, y_min, z_min), glm::vec3(x_max, y_max, z_max) };
			leaves.push_back(node);
		}
	}

	std::sort(leaves.begin(), leaves.end());
	build(leaves, 0);
}

void BoundingVolumeHierarchy::build(std::vector<Node> children, int depth)
{
	Node node;
	node.depth = depth;
	if (levels < depth + 1) {
		levels = depth + 1;
	}
	if (children.size() == 1) {
		node.isLeaf = true;
		node.aabb = children.front().aabb;
		nodes.push_back(node);
	}
	else {
		node.aabb = getAABB(children);
		nodes.push_back(node);

		std::vector<Node> left_nodes(children.begin(), children.begin() + children.size() / 2);
		std::vector<Node> right_nodes(children.begin() + children.size() / 2, children.end());

		build(left_nodes, depth + 1);
		build(right_nodes, depth + 1);
	}
}

AxisAlignedBox BoundingVolumeHierarchy::getAABB(std::vector<Node> nodes) {
	float x_min = std::numeric_limits<float>::max();
	float y_min = std::numeric_limits<float>::max();
	float z_min = std::numeric_limits<float>::max();

	float x_max = std::numeric_limits<float>::min();
	float y_max = std::numeric_limits<float>::min();
	float z_max = std::numeric_limits<float>::min();


	for (Node node : nodes) {
		if (node.aabb.lower.x < x_min) {
			x_min = node.aabb.lower.x;
		}

		if (node.aabb.lower.y < y_min) {
			y_min = node.aabb.lower.y;
		}

		if (node.aabb.lower.z < z_min) {
			z_min = node.aabb.lower.z;
		}

		if (node.aabb.upper.x > x_max) {
			x_max = node.aabb.upper.x;
		}

		if (node.aabb.upper.y > y_max) {
			y_max = node.aabb.upper.y;
		}

		if (node.aabb.upper.z > z_max) {
			z_max = node.aabb.upper.z;
		}
	}
	return { glm::vec3(x_min, y_min, z_min), glm::vec3(x_max, y_max, z_max) };
}


// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{
	for (Node node : nodes) {
		if (node.depth == level) {
			drawAABB(node.aabb, DrawMode::Wireframe, glm::vec3(0.05f, 1.05f, 1.05f), 0.5f);
		}
	}
}

int BoundingVolumeHierarchy::numLevels()
{
	return levels;
}


// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h .
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo) const
{
	bool hit = false;
	float t = ray.t;
	// Intersect with all triangles of all meshes.
	for (const auto& mesh : m_pScene->meshes) {
		for (const auto& tri : mesh.triangles) {
			const auto v0 = mesh.vertices[tri[0]];
			const auto v1 = mesh.vertices[tri[1]];
			const auto v2 = mesh.vertices[tri[2]];
			hit |= intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo);
			if (ray.t < t) {
				hitInfo.normal = glm::normalize(glm::cross((v1.p - v0.p), (v2.p - v0.p)));
				hitInfo.material = mesh.material;
				t = ray.t;
			}
		}
	}
	ray.t = t;

	// Intersect with spheres.
	for (const auto& sphere : m_pScene->spheres)
		hit |= intersectRayWithShape(sphere, ray, hitInfo);
	return hit;
}
