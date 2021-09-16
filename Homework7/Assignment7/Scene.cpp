//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::build(BVHAccel::SplitMethod Method)
{
    switch (Method)
    {
    case BVHAccel::SplitMethod::NAIVE:
        this->buildBVH();
        break;
    case BVHAccel::SplitMethod::SAH:
        this->buildSAH();
        break;
    default:
        break;
    }
}

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

void Scene::buildSAH()
{
    printf(" - Generating SAH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
	// 只对含自发光物体sample
	float emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	float p = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
			if (p <= emit_area_sum) {
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    if (depth > maxDepth)
    {
        return {};
    }

    Intersection objectInter = intersect(ray);
    if (!objectInter.happened)
    {
        return {};
    }

    if (objectInter.m->hasEmission())
    {
        return objectInter.m->getEmission();
    }

	Vector3f L_dir = { 0,0,0 };
	Vector3f L_indir = { 0,0,0 };

	Intersection lightInter;
	float m_pdf = 0.0;
	sampleLight(lightInter, m_pdf);

	Vector3f obj2Light = lightInter.coords - objectInter.coords;
	Vector3f obj2LightDir = obj2Light.normalized();

	float distance = obj2Light.norm();
    float sqrMagnitude = obj2Light.x * obj2Light.x + obj2Light.y * obj2Light.y + obj2Light.y + obj2Light.z * obj2Light.z;

	auto m_emit = lightInter.emit;
	auto m_eval = objectInter.m->eval(ray.direction, obj2LightDir, objectInter.normal);

	// Get x, ws, NN, emit from inter
	// Shoot a ray from p to x
    // x 光源点
    // p 物体着色点
    // wo 往像素发射的入射光线
    // ws 物体往光源发射的射线
    // N objectInter法线向量
    // NN lightInter法线向量

    Ray obj2LightRay{ objectInter.coords, obj2LightDir };
    Intersection Light = intersect(obj2LightRay);

    if (Light.distance - distance >= -EPSILON)
    {
        L_dir = m_emit * m_eval * dotProduct(obj2LightDir, objectInter.normal) * dotProduct(-obj2LightDir, lightInter.normal) / sqrMagnitude / m_pdf;
    }

    if (get_random_float() >= RussianRoulette)
    {
        return L_dir;
    }

    Vector3f objRSample = objectInter.m->sample(ray.direction, objectInter.normal);
    Ray obj2NextObjRay{ objectInter.coords, objRSample.normalized() };
    Intersection nextObjIntersection = intersect(obj2NextObjRay);

	if (nextObjIntersection.happened && !nextObjIntersection.m->hasEmission())
	{
		m_pdf = objectInter.m->pdf(ray.direction, objRSample.normalized(), objectInter.normal);
        Vector3f result = castRay(obj2NextObjRay, depth + 1);
        result.x = clamp(0, 1, result.x);
        result.y = clamp(0, 1, result.y);
        result.z = clamp(0, 1, result.z);
		L_indir = result * objectInter.m->eval(ray.direction, objRSample.normalized(), objectInter.normal)
            * dotProduct(objRSample.normalized(), objectInter.normal) / m_pdf / RussianRoulette;
	}

    return L_dir + L_indir;
}