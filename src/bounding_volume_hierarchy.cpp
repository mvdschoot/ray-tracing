#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
#include <iostream>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene, const int MAX_BVH_LEVEL)
	: m_pScene(pScene)
{
	this->MAX_BVH_LEVEL = MAX_BVH_LEVEL;
	std::vector<Primitive> primitives;

	for (int i = 0; i != m_pScene->meshes.size(); i++) {
		Mesh mesh = m_pScene->meshes[i];
		for (const auto& tri : mesh.triangles) {
			const auto& v0 = mesh.vertices[tri[0]];
			const auto& v1 = mesh.vertices[tri[1]];
			const auto& v2 = mesh.vertices[tri[2]];

			float x_min = glm::min(v0.p.x, glm::min(v1.p.x, v2.p.x));
			float y_min = glm::min(v0.p.y, glm::min(v1.p.y, v2.p.y));
			float z_min = glm::min(v0.p.z, glm::min(v1.p.z, v2.p.z));

			float x_max = glm::max(v0.p.x, glm::max(v1.p.x, v2.p.x));
			float y_max = glm::max(v0.p.y, glm::max(v1.p.y, v2.p.y));
			float z_max = glm::max(v0.p.z, glm::max(v1.p.z, v2.p.z));

			Primitive primitive;
			primitive.aabb = { glm::vec3(x_min, y_min, z_min), glm::vec3(x_max, y_max, z_max) };
			primitive.triangle = tri;
			primitive.mesh_idx = i;
			primitives.push_back(primitive);
		}
	}

	std::sort(primitives.begin(), primitives.end());
	countLevels(primitives, 0);
	Node empty;
	empty.depth = -1;
	nodes.resize(glm::pow(2, levels) - 1, empty);
	build(primitives, 0, 0);
}

void BoundingVolumeHierarchy::countLevels(std::vector<Primitive> primitives, int depth)
{
	if (levels < depth + 1) {
		levels = depth + 1;
	}
	if (primitives.size() != 1 && depth != MAX_BVH_LEVEL) {
		std::vector<Primitive> left_nodes(primitives.begin(), primitives.begin() + primitives.size() / 2);
		std::vector<Primitive> right_nodes(primitives.begin() + primitives.size() / 2, primitives.end());
		countLevels(left_nodes, depth + 1);
		countLevels(right_nodes, depth + 1);
	}
}

void BoundingVolumeHierarchy::build(std::vector<Primitive> primitives, int depth, int idx)
{
	Node node;
	node.depth = depth;
	if (primitives.size() == 1 || depth == MAX_BVH_LEVEL) {
		node.isLeaf = true;
		node.aabb = getAABB(primitives);
		node.primitives = primitives;
		node.idx = idx;
		nodes[idx] = node;
	}
	else {
		node.aabb = getAABB(primitives);
		node.idx = idx;
		nodes[idx] = node;

		std::vector<Primitive> left_nodes(primitives.begin(), primitives.begin() + primitives.size() / 2);
		std::vector<Primitive> right_nodes(primitives.begin() + primitives.size() / 2, primitives.end());

		build(left_nodes, depth + 1, 2 * idx + 1);
		build(right_nodes, depth + 1, 2 * idx + 2);
	}
}

AxisAlignedBox BoundingVolumeHierarchy::getAABB(std::vector<Primitive> primitives) {
	float x_min = std::numeric_limits<float>::max();
	float y_min = std::numeric_limits<float>::max();
	float z_min = std::numeric_limits<float>::max();

	float x_max = std::numeric_limits<float>::min();
	float y_max = std::numeric_limits<float>::min();
	float z_max = std::numeric_limits<float>::min();


	for (Primitive primitive : primitives) {
		if (primitive.aabb.lower.x < x_min) {
			x_min = primitive.aabb.lower.x;
		}

		if (primitive.aabb.lower.y < y_min) {
			y_min = primitive.aabb.lower.y;
		}

		if (primitive.aabb.lower.z < z_min) {
			z_min = primitive.aabb.lower.z;
		}

		if (primitive.aabb.upper.x > x_max) {
			x_max = primitive.aabb.upper.x;
		}

		if (primitive.aabb.upper.y > y_max) {
			y_max = primitive.aabb.upper.y;
		}

		if (primitive.aabb.upper.z > z_max) {
			z_max = primitive.aabb.upper.z;
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
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, bool interpolate) const
{
	bool hit = false;
	float t = ray.t;
	// Intersect with all triangles of all meshes.
	for (const auto& mesh : m_pScene->meshes) {
		for (const auto& tri : mesh.triangles) {
			const auto& v0 = mesh.vertices[tri[0]];
			const auto& v1 = mesh.vertices[tri[1]];
			const auto& v2 = mesh.vertices[tri[2]];
			hit |= intersectRayWithTriangle(v0.p, v1.p, v2.p, ray, hitInfo);
			if (ray.t < t && t > 0) {
				if (interpolate) {
					glm::vec3 p = ray.t * ray.direction + ray.origin;
					float total_area = glm::length(glm::cross(v1.p - v0.p, v2.p - v0.p)) / 2.0f;
					float alpha = (glm::length(glm::cross(v1.p - p, v2.p - p)) / 2.0f) / total_area;
					float beta = (glm::length(glm::cross(v0.p - p, v2.p - p)) / 2.0f) / total_area;
					float gamma = (glm::length(glm::cross(v0.p - p, v1.p - p)) / 2.0f) / total_area;
					hitInfo.normal = glm::normalize(v0.n * alpha + v1.n * beta + v2.n * gamma);
				}
				else {
					hitInfo.normal = glm::normalize(glm::cross((v1.p - v0.p), (v2.p - v0.p)));
				}
				hitInfo.material = mesh.material;
				t = ray.t;
			}
		}
	}
	ray.t = t;

	// Intersect with spheres.
	for (const auto& sphere : m_pScene->spheres) {
		bool tempHit = intersectRayWithShape(sphere, ray, hitInfo);
		if (tempHit) {
			hitInfo.normal = glm::normalize((ray.origin + ray.direction * ray.t) - sphere.center);
			hitInfo.material = sphere.material;
			hit = true;
		}
	}
	return hit;
}
