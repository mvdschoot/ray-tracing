#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
#include <iostream>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
	: m_pScene(pScene)
{
}

// Use this function to visualize your BVH. This can be useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDraw(int level)
{

	// Draw the AABB as a transparent green box.
	//AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
	//drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

	// Draw the AABB as a (white) wireframe box.
	AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
	//drawAABB(aabb, DrawMode::Wireframe);
	drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);
}

int BoundingVolumeHierarchy::numLevels() const
{
	return 5;
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
