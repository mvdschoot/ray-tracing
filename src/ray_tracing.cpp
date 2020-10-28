#include "ray_tracing.h"
#include "disable_all_warnings.h"
// Suppress warnings in third-party code.
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>

bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{
	return (
		glm::dot(n, glm::cross(v1 - v0, p - v0)) >= 0 &&
		glm::dot(n, glm::cross(v2 - v1, p - v1)) >= 0 &&
		glm::dot(n, glm::cross(v0 - v2, p - v2)) >= 0
		);
}

bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
	float t = (-plane.D - glm::dot(ray.origin, plane.normal)) / glm::dot(ray.direction, plane.normal);
	if (glm::abs(glm::dot(plane.normal, ray.direction)) < 1e-6 && t != 0 || t < 0) {
		return false;
	}
	ray.t = t;
	return true;
}

Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
	Plane plane;
	plane.normal = glm::cross((v1 - v0), (v2 - v0));
	plane.D = -glm::dot(v0, plane.normal);
	return plane;
}

/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{
	Plane plane = trianglePlane(v0, v1, v2);
	if (intersectRayWithPlane(plane, ray) && pointInTriangle(v0, v1, v2, plane.normal, ray.origin + ray.direction * ray.t)) {
		return true;
	}
	ray.t = std::numeric_limits<float>::max();
	return false;
}

/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{
	float a = glm::pow(ray.direction.x, 2) + glm::pow(ray.direction.y, 2) + glm::pow(ray.direction.z, 2);
	float b = 2 * (ray.direction.x * (ray.origin.x - sphere.center.x) + ray.direction.y * (ray.origin.y - sphere.center.y) + ray.direction.z * (ray.origin.z - sphere.center.z));
	float c = glm::pow(ray.origin.x - sphere.center.x, 2) + glm::pow(ray.origin.y - sphere.center.y, 2) + glm::pow(ray.origin.z - sphere.center.z, 2) - glm::pow(sphere.radius, 2);

	float discriminant = glm::pow(b, 2) - 4 * a * c;

	if (discriminant < 0) {
		return false;
	}

	float t_in = (-b - glm::sqrt(discriminant)) / (2 * a);
	float t_out = (-b + glm::sqrt(discriminant)) / (2 * a);

	float t_min = glm::min(t_in, t_out);

	if (t_min < ray.t) {
		ray.t = t_min;
	}

	return true;
}

/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
	float tx_min = (box.lower.x - ray.origin.x) / ray.direction.x;
	float tx_max = (box.upper.x - ray.origin.x) / ray.direction.x;
	float ty_min = (box.lower.y - ray.origin.y) / ray.direction.y;
	float ty_max = (box.upper.y - ray.origin.y) / ray.direction.y;
	float tz_min = (box.lower.z - ray.origin.z) / ray.direction.z;
	float tz_max = (box.upper.z - ray.origin.z) / ray.direction.z;

	float tx_in = glm::min(tx_min, tx_max);
	float tx_out = glm::max(tx_min, tx_max);
	float ty_in = glm::min(ty_min, ty_max);
	float ty_out = glm::max(ty_min, ty_max);
	float tz_in = glm::min(tz_min, tz_max);
	float tz_out = glm::max(tz_min, tz_max);

	float t_in = glm::max(tx_in, glm::max(ty_in, tz_in));
	float t_out = glm::min(tx_out, glm::min(ty_out, tz_out));

	if (t_in > t_out || t_out < 0) {
		return false;
	}

	if (t_in < ray.t) {
		ray.t = t_in;
	}

	return true;
}
