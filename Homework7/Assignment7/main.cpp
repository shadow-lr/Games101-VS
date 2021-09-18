#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <omp.h>
#include <chrono>
#include <direct.h>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
	// Change the definition here to change resolution
	omp_init_lock(&lock);
	Scene scene(1024, 1024);

	// Vector3f(0.0f)是否是自发光
	Material* red = new Material(DIFFUSE, Vector3f(0.0f));
	red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
	Material* green = new Material(DIFFUSE, Vector3f(0.0f));
	green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
	Material* white = new Material(DIFFUSE, Vector3f(0.0f));
	white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
	Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) + 15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) + 18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
	light->Kd = Vector3f(0.65f);

	Material* glass = new Material(Microfacet, Vector3f(0.0f));
	glass->Kd = Vector3f(0.3f, 0.3f, 0.25f);
	glass->Ks = Vector3f(0.8f, 0.8f, 0.8f);

	Material* silver = new Material(MicrofacetGlossy, Vector3f(0.0f));
	silver->Kd = Vector3f(1.0f, 0.5f, 0.31f);
	silver->Ks = Vector3f(0.6f, 0.6f, 0.6f);

	BVHAccel::SplitMethod splitMethod = BVHAccel::SplitMethod::NAIVE;

	MeshTriangle floor("./Homework7/Assignment7/models/cornellbox/floor.obj", white, splitMethod);
	//MeshTriangle shortbox("./Homework7/Assignment7/models/cornellbox/shortbox.obj", white, splitMethod);
	MeshTriangle tallbox("./Homework7/Assignment7/models/cornellbox/tallbox.obj", white, splitMethod);
	MeshTriangle left("./Homework7/Assignment7/models/cornellbox/left.obj", red, splitMethod);
	MeshTriangle right("./Homework7/Assignment7/models/cornellbox/right.obj", green, splitMethod);
	MeshTriangle light_("./Homework7/Assignment7/models/cornellbox/light.obj", light, splitMethod);

	std::array<float, 3> translate = { 450.0f,0.0f,150.0f };
	std::array<float, 3> scale = { 1500.0f,1500.0f,1500.0f };
	MeshTriangle bunny("./Homework7/Assignment7/models/bunny/bunny.obj", white, splitMethod, translate, scale);

	Sphere sphere(Vector3f(150.0f, 100.0f, 300.0f), 100, glass);
	Sphere sphere1(Vector3f(350.0f, 100.0f, 400.0f), 100, silver);

	scene.Add(&floor);
	//scene.Add(&shortbox);
	//scene.Add(&tallbox);
	scene.Add(&left);
	scene.Add(&right);
	scene.Add(&light_);
	scene.Add(&sphere);
	scene.Add(&sphere1);
	scene.Add(&bunny);

	scene.build(splitMethod);

	Renderer r;

	auto start = std::chrono::system_clock::now();
	r.Render(scene);
	auto stop = std::chrono::system_clock::now();

	std::cout << "Render complete: \n";
	std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
	std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
	std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

	omp_destroy_lock(&lock);

	return 0;
}