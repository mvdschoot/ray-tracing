#include "bounding_volume_hierarchy.h"
#include "disable_all_warnings.h"
#include "draw.h"
#include "image.h"
#include "ray_tracing.h"
#include "screen.h"
#include "trackball.h"
#include "window.h"
#include <thread>
// Disable compiler warnings in third-party code (which we cannot change).
DISABLE_WARNINGS_PUSH()
#include <glm/gtc/type_ptr.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <imgui.h>
DISABLE_WARNINGS_POP()
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <type_traits>
#ifdef USE_OPENMP
#include <omp.h>
#endif

// This is the main application. The code in here does not need to be modified.
constexpr glm::ivec2 windowResolution{ 800, 800 };
const std::filesystem::path dataPath{ DATA_DIR };
const std::filesystem::path outputPath{ OUTPUT_DIR };

// Constants
const int MAX_BVH_LEVEL = 15;
const int RECURSION_DEPTH = 3;
const std::vector<std::string> N_THREAD_VALUES{ "1", "2", "5", "10", "20" };

// Ray Tracing options
bool recursive = true;
bool interpolate = false;
bool shadows = true;
int n_threads_idx = 0;

enum class ViewMode {
	Rasterization = 0,
	RayTracing = 1
};


glm::vec3 diffuse(const Material& material, const glm::vec3& vertexPos, const glm::vec3& normal, const glm::vec3& lightPos)
{
	glm::vec3 diffuse = material.kd * glm::dot(normal, glm::normalize(lightPos - vertexPos));
	return glm::vec3(glm::max(diffuse.x, 0.0f), glm::max(diffuse.y, 0.0f), glm::max(diffuse.z, 0.0f));
}


glm::vec3 specular(const Material& material, const glm::vec3& vertexPos, const glm::vec3& normal, const glm::vec3& lightPos, const glm::vec3& cameraPos)
{
	glm::vec3 L = glm::normalize(lightPos - vertexPos);
	glm::vec3 R = glm::normalize(glm::reflect(L, normal));
	glm::vec3 V = glm::normalize(cameraPos - vertexPos);
	glm::vec3 specular = material.ks * glm::pow(glm::max(glm::dot(R, V), 0.0f), material.shininess);
	return glm::vec3(glm::max(specular.x, 0.0f), glm::max(specular.y, 0.0f), glm::max(specular.z, 0.0f));

}

static glm::vec3 colorPointLight(const PointLight& pointLight, const BoundingVolumeHierarchy& bvh, Ray ray, HitInfo hitInfo) {
	glm::vec3 color(0.0f);

	glm::vec3 intersectPoint = ray.origin + ray.direction * ray.t;
	glm::vec3 vToLight = pointLight.position - intersectPoint;
	Ray toLight{ intersectPoint + vToLight * 0.001f, vToLight };

	HitInfo inf;
	bool intersect = bvh.intersect(toLight, inf, interpolate, 0);
	bool right = rightSideOfPlane(ray, toLight, hitInfo.normal);

	if ((toLight.t > 1 && right) || !shadows) {
		toLight.t = 1.0f;
		if (shadows != (recursive && hitInfo.material.ks != glm::vec3(0.0f))) {
			drawRay(toLight);
		}

		color += diffuse(hitInfo.material, ray.origin + ray.direction * ray.t, hitInfo.normal, pointLight.position);
		color += specular(hitInfo.material, ray.origin + ray.direction * ray.t, hitInfo.normal, pointLight.position, ray.origin);
		const glm::vec3 diff = pointLight.position - (ray.origin + ray.direction * ray.t);
		const float dist2 = glm::dot(diff, diff);
		const glm::vec3 Li = pointLight.color / glm::max(dist2, 1.0f);
		color *= Li;
	}
	else {
		if (toLight.t > 1) {
			toLight.t = 1;
		}
		if (shadows != (recursive && hitInfo.material.ks != glm::vec3(0.0f))) {
			drawRay(toLight, glm::vec3{ 1.0f,0.0f,0.0f });
		}
	}
	return color;
}

static glm::vec3 calculateColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray, HitInfo hitInfo)
{

	glm::vec3 color = glm::vec3(0.0f);


	for (const PointLight& pointLight : scene.pointLights) {
		color += colorPointLight(pointLight, bvh, ray, hitInfo);
	}

	for (const SphericalLight& sLight : scene.sphericalLight) {
		glm::vec3 sumSphereColors(0.0f);
		glm::vec3 origin = ray.origin + ray.direction * ray.t;
		auto points = getSpherePoints(Sphere{ sLight.position, sLight.radius, Material{} }, origin);

		for (glm::vec3 point : points) {
			PointLight light{ point, sLight.color };
			sumSphereColors += colorPointLight(light, bvh, ray, hitInfo);
		}
		sumSphereColors /= samples;
		color += sumSphereColors;
	}

	return color;
}


static glm::vec3 getFinalColorRecursive(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray, int depth)
{
	HitInfo hitInfo;
	glm::vec3 color = glm::vec3(0.0f);

	float bias = 0.00001f;
	if (bvh.intersect(ray, hitInfo, interpolate, 0)) {
		if (hitInfo.material.ks != glm::vec3(0.0f) && depth++ < RECURSION_DEPTH && recursive) {
			Ray reflectedRay;
			reflectedRay.direction = ray.direction - hitInfo.normal * glm::dot(hitInfo.normal, ray.direction) * 2.0f;
			reflectedRay.origin = (ray.origin + ray.direction * ray.t) + reflectedRay.direction * bias;
			color = calculateColor(scene, bvh, ray, hitInfo) + getFinalColorRecursive(scene, bvh, reflectedRay, depth);
		}
		else {
			color = calculateColor(scene, bvh, ray, hitInfo);
		}
		drawRay(ray, color);
	}
	else {
		drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
	}
	return color;
}


static glm::vec3 getFinalColor(const Scene& scene, const BoundingVolumeHierarchy& bvh, Ray ray)
{
	return getFinalColorRecursive(scene, bvh, ray, 0);
}

static void renderRayTracing_thread(int start, int stop, const Scene& scene, const Trackball& camera, const BoundingVolumeHierarchy& bvh, Screen& screen)
{
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
	for (int y = start; y < stop; y++) {
		for (int x = 0; x != windowResolution.x; x++) {
			const glm::vec2 normalizedPixelPos{
				float(x) / windowResolution.x * 2.0f - 1.0f,
				float(y) / windowResolution.y * 2.0f - 1.0f
			};
			const Ray cameraRay = camera.generateRay(normalizedPixelPos);
			glm::vec3 color = getFinalColor(scene, bvh, cameraRay);
			screen.setPixel(x, y, color);
		}
	}
}

static void setOpenGLMatrices(const Trackball& camera);
static void renderOpenGL(const Scene& scene, const Trackball& camera, int selectedLight);

static void renderRayTracing(const Scene& scene, const Trackball& camera, const BoundingVolumeHierarchy& bvh, Screen& screen)
{
	std::vector<std::thread> threads;

	int start = 0;
	int offset = windowResolution.y;
	int n_threads = std::stoi(N_THREAD_VALUES[n_threads_idx]);

	if (n_threads > 1 && windowResolution.y % n_threads == 0) {
		offset /= n_threads;
	}

	for (int i = 0; i < n_threads; i++) {
		threads.emplace_back(renderRayTracing_thread, start, start + offset, std::ref(scene), std::ref(camera), std::ref(bvh), std::ref(screen));
		start += offset;
	}

	for (std::thread& t : threads) {
		t.join();
	}
}

int main(int argc, char** argv)
{
	Trackball::printHelp();
	std::cout << "\n Press the [R] key on your keyboard to create a ray towards the mouse cursor" << std::endl
		<< std::endl;

	Window window{ "Final Project - Part 2", windowResolution, OpenGLVersion::GL2 };
	Screen screen{ windowResolution };
	Trackball camera{ &window, glm::radians(50.0f), 3.0f };
	camera.setCamera(glm::vec3(0.0f, 0.0f, 0.0f), glm::radians(glm::vec3(20.0f, 20.0f, 0.0f)), 3.0f);

	SceneType sceneType{ SceneType::SingleTriangle };
	std::optional<Ray> optDebugRay;
	Scene scene = loadScene(sceneType, dataPath);
	BoundingVolumeHierarchy bvh{ &scene , MAX_BVH_LEVEL };

	int bvhDebugLevel = 0;
	bool debugBVH{ false };
	ViewMode viewMode{ ViewMode::Rasterization };

	window.registerKeyCallback([&](int key, int /* scancode */, int action, int /* mods */) {
		if (action == GLFW_PRESS) {
			switch (key) {
			case GLFW_KEY_R: {
				// Shoot a ray. Produce a ray from camera to the far plane.
				const auto tmp = window.getNormalizedCursorPos();
				optDebugRay = camera.generateRay(tmp * 2.0f - 1.0f);
				viewMode = ViewMode::Rasterization;
			} break;
			case GLFW_KEY_ESCAPE: {
				window.close();
			} break;
			};
		}
	});

	int selectedLight{ 0 };
	while (!window.shouldClose()) {
		window.updateInput();

		// === Setup the UI ===
		ImGui::Begin("Final Project - Part 2");
		{
			constexpr std::array items{ "SingleTriangle", "Cube", "Cornell Box (with mirror)", "Cornell Box (spherical light and mirror)", "Monkey", "Teapot", "Dragon", /* "AABBs",*/ "Spheres", /*"Mixed",*/ "Custom" };
			if (ImGui::Combo("Scenes", reinterpret_cast<int*>(&sceneType), items.data(), int(items.size()))) {
				optDebugRay.reset();
				scene = loadScene(sceneType, dataPath);
				selectedLight = 0;
				bvh = BoundingVolumeHierarchy(&scene, MAX_BVH_LEVEL);
				if (optDebugRay) {
					HitInfo dummy{};
					bvh.intersect(*optDebugRay, dummy, interpolate, 0);
				}
			}
		}
		{
			constexpr std::array items{ "Rasterization", "Ray Traced" };
			ImGui::Combo("View mode", reinterpret_cast<int*>(&viewMode), items.data(), int(items.size()));
		}
		if (ImGui::Button("Render to file")) {
			{
				using clock = std::chrono::high_resolution_clock;
				const auto start = clock::now();
				renderRayTracing(scene, camera, bvh, screen);
				const auto end = clock::now();

				int n_triangles = 0;

				for (const auto& mesh : scene.meshes) {
					n_triangles += mesh.triangles.size();
				}

				int ms = std::chrono::duration<float, std::milli>(end - start).count();
				int h = ms / (1000 * 60 * 60);
				ms -= h * (1000 * 60 * 60);
				int m = ms / (1000 * 60);
				ms -= m * (1000 * 60);
				int s = ms / 1000;
				ms -= s * 1000;

				std::cout << "\nTime to render image: " << std::setfill('0') << std::setw(2) << h << ':' << std::setw(2) << m << ':' << std::setw(2) << s << '.' << std::setw(3) << ms << std::endl;
				std::cout << "Number of triangles: " << n_triangles << std::endl;
				std::cout << "Number of BVH levels: " << bvh.levels << std::endl;
				std::cout << "- Recursive: " << (recursive ? "yes" : "no") << std::endl;
				std::cout << "- Interpolated normals: " << (interpolate ? "yes" : "no") << std::endl;
				std::cout << "- Shadows: " << (shadows ? "yes" : "no") << std::endl;
				std::cout << "- Number of threads: " << N_THREAD_VALUES[n_threads_idx] << std::endl;
			}
			screen.writeBitmapToFile(outputPath / "render.bmp");
		}
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Text("Debugging");
		if (viewMode == ViewMode::Rasterization) {
			ImGui::Checkbox("Draw BVH", &debugBVH);
			if (debugBVH)
				ImGui::SliderInt("BVH Level", &bvhDebugLevel, 0, bvh.numLevels() - 1);
		}

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Text("Ray Tracing");
		ImGui::Checkbox("Recursive Ray Tracing", &recursive);
		ImGui::Checkbox("Interpolate Normals", &interpolate);
		ImGui::Checkbox("Shadows", &shadows);

		std::vector<const char*> optionsPointers;
		std::transform(std::begin(N_THREAD_VALUES), std::end(N_THREAD_VALUES), std::back_inserter(optionsPointers),
			[](const auto& str) { return str.c_str(); });
		ImGui::Combo("Number of Threads", &n_threads_idx, optionsPointers.data(), static_cast<int>(optionsPointers.size()));

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Text("Lights");
		if (!scene.pointLights.empty() || !scene.sphericalLight.empty()) {
			{
				std::vector<std::string> options;
				for (size_t i = 0; i < scene.pointLights.size(); i++) {
					options.push_back("Point Light " + std::to_string(i + 1));
				}
				for (size_t i = 0; i < scene.sphericalLight.size(); i++) {
					options.push_back("Spherical Light " + std::to_string(i + 1));
				}

				std::vector<const char*> optionsPointers;
				std::transform(std::begin(options), std::end(options), std::back_inserter(optionsPointers),
					[](const auto& str) { return str.c_str(); });

				ImGui::Combo("Selected light", &selectedLight, optionsPointers.data(), static_cast<int>(optionsPointers.size()));
			}

			{
				const auto showLightOptions = [](auto& light) {
					ImGui::DragFloat3("Light position", glm::value_ptr(light.position), 0.01f, -3.0f, 3.0f);
					ImGui::ColorEdit3("Light color", glm::value_ptr(light.color));
					if constexpr (std::is_same_v<std::decay_t<decltype(light)>, SphericalLight>) {
						ImGui::DragFloat("Light radius", &light.radius, 0.01f, 0.01f, 0.5f);
					}
				};
				if (selectedLight < static_cast<int>(scene.pointLights.size())) {
					// Draw a big yellow sphere and then the small light sphere on top.
					showLightOptions(scene.pointLights[selectedLight]);
				}
				else {
					// Draw a big yellow sphere and then the smaller light sphere on top.
					showLightOptions(scene.sphericalLight[selectedLight - scene.pointLights.size()]);
				}
			}
		}

		if (ImGui::Button("Add point light")) {
			scene.pointLights.push_back(PointLight{ glm::vec3(0.0f), glm::vec3(1.0f) });
			selectedLight = int(scene.pointLights.size() - 1);
		}
		if (ImGui::Button("Add spherical light")) {
			scene.sphericalLight.push_back(SphericalLight{ glm::vec3(0.0f), 0.1f, glm::vec3(1.0f) });
			selectedLight = int(scene.pointLights.size() + scene.sphericalLight.size() - 1);
		}
		if (ImGui::Button("Remove selected light")) {
			if (selectedLight < static_cast<int>(scene.pointLights.size())) {
				scene.pointLights.erase(std::begin(scene.pointLights) + selectedLight);
			}
			else {
				scene.sphericalLight.erase(std::begin(scene.sphericalLight) + (selectedLight - scene.pointLights.size()));
			}
			selectedLight = 0;
		}

		// Clear screen.
		glClearDepth(1.0f);
		glClearColor(0.0, 0.0, 0.0, 0.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Draw either using OpenGL (rasterization) or the ray tracing function.
		switch (viewMode) {
		case ViewMode::Rasterization: {
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			renderOpenGL(scene, camera, selectedLight);
			if (optDebugRay) {
				// Call getFinalColor for the debug ray. Ignore the result but tell the function that it should
				// draw the rays instead.
				enableDrawRay = true;
				(void)getFinalColor(scene, bvh, *optDebugRay);
				enableDrawRay = false;
			}
			glPopAttrib();
		} break;
		case ViewMode::RayTracing: {
			screen.clear(glm::vec3(0.0f));
			renderRayTracing(scene, camera, bvh, screen);
			screen.setPixel(0, 0, glm::vec3(1.0f));
			screen.draw(); // Takes the image generated using ray tracing and outputs it to the screen using OpenGL.
		} break;
		default:
			break;
		};

		if (debugBVH) {
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			setOpenGLMatrices(camera);
			glDisable(GL_LIGHTING);
			glEnable(GL_DEPTH_TEST);

			// Enable alpha blending. More info at:
			// https://learnopengl.com/Advanced-OpenGL/Blending
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			bvh.debugDraw(bvhDebugLevel);
			glPopAttrib();
		}

		ImGui::End();
		window.swapBuffers();
	}

	return 0; // execution never reaches this point
}

static void setOpenGLMatrices(const Trackball& camera)
{
	// Load view matrix.
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	const glm::mat4 viewMatrix = camera.viewMatrix();
	glMultMatrixf(glm::value_ptr(viewMatrix));

	// Load projection matrix.
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	const glm::mat4 projectionMatrix = camera.projectionMatrix();
	glMultMatrixf(glm::value_ptr(projectionMatrix));
}

static void renderOpenGL(const Scene& scene, const Trackball& camera, int selectedLight)
{
	// Normals will be normalized in the graphics pipeline.
	glEnable(GL_NORMALIZE);
	// Activate rendering modes.
	glEnable(GL_DEPTH_TEST);
	// Draw front and back facing triangles filled.
	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_FILL);
	// Interpolate vertex colors over the triangles.
	glShadeModel(GL_SMOOTH);
	setOpenGLMatrices(camera);

	glDisable(GL_LIGHTING);
	// Render point lights as very small dots
	for (const auto& light : scene.pointLights)
		drawSphere(light.position, 0.01f, light.color);
	for (const auto& light : scene.sphericalLight)
		drawSphere(light.position, light.radius, light.color);

	if (!scene.pointLights.empty() || !scene.sphericalLight.empty()) {
		if (selectedLight < static_cast<int>(scene.pointLights.size())) {
			// Draw a big yellow sphere and then the small light sphere on top.
			const auto& light = scene.pointLights[selectedLight];
			drawSphere(light.position, 0.05f, glm::vec3(1, 1, 0));
			glDisable(GL_DEPTH_TEST);
			drawSphere(light.position, 0.01f, light.color);
			glEnable(GL_DEPTH_TEST);
		}
		else {
			// Draw a big yellow sphere and then the smaller light sphere on top.
			const auto& light = scene.sphericalLight[selectedLight - scene.pointLights.size()];
			drawSphere(light.position, light.radius + 0.01f, glm::vec3(1, 1, 0));
			glDisable(GL_DEPTH_TEST);
			drawSphere(light.position, light.radius, light.color);
			glEnable(GL_DEPTH_TEST);
		}
	}

	// Activate the light in the legacy OpenGL mode.
	glEnable(GL_LIGHTING);

	int i = 0;
	const auto enableLight = [&](const auto& light) {
		glEnable(GL_LIGHT0 + i);
		const glm::vec4 position4{ light.position, 1 };
		glLightfv(GL_LIGHT0 + i, GL_POSITION, glm::value_ptr(position4));
		const glm::vec4 color4{ glm::clamp(light.color, 0.0f, 1.0f), 1.0f };
		const glm::vec4 zero4{ 0.0f, 0.0f, 0.0f, 1.0f };
		glLightfv(GL_LIGHT0 + i, GL_AMBIENT, glm::value_ptr(zero4));
		glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, glm::value_ptr(color4));
		glLightfv(GL_LIGHT0 + i, GL_SPECULAR, glm::value_ptr(zero4));
		// NOTE: quadratic attenuation doesn't work like you think it would in legacy OpenGL.
		// The distance is not in world space but in NDC space!
		glLightf(GL_LIGHT0 + i, GL_CONSTANT_ATTENUATION, 1.0f);
		glLightf(GL_LIGHT0 + i, GL_LINEAR_ATTENUATION, 0.0f);
		glLightf(GL_LIGHT0 + i, GL_QUADRATIC_ATTENUATION, 0.0f);
		i++;
	};
	for (const auto& light : scene.pointLights)
		enableLight(light);
	for (const auto& light : scene.sphericalLight)
		enableLight(light);

	// Draw the scene and the ray (if any).
	drawScene(scene);

	// Draw a colored sphere at the location at which the trackball is looking/rotating around.
	glDisable(GL_LIGHTING);
	drawSphere(camera.lookAt(), 0.01f, glm::vec3(0.2f, 0.2f, 1.0f));
}
