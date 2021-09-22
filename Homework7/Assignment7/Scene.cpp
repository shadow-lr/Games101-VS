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

	auto format = [](Vector3f& a) {
		if (a.x < 0) a.x = 0;
		if (a.y < 0) a.y = 0;
		if (a.z < 0) a.z = 0;
	};

    Intersection pos = intersect(ray);
    if (!pos.happened)
    {
        return {};
    }

    if (pos.m->hasEmission())
    {
        return pos.m->getEmission();
    }

	Vector3f L_dir = { 0,0,0 };
	Vector3f L_indir = { 0,0,0 };

	Intersection light;
	float m_pdf = 0.0;
	sampleLight(light, m_pdf);

	// pos
	Vector3f normal = pos.normal;

	Vector3f wo = light.coords - pos.coords;
	Vector3f obj2LightDir = pos2Light.normalized();

	// Get x, ws, NN, emit from inter
	// Shoot a ray from p to x
	// x 光源点
	// p 物体着色点
	// wo 往像素发射的入射光线
	// ws 物体往光源发射的射线
	// N objectInter法线向量
	// NN lightInter法线向量

	float distance = pos2Light.norm();
    float sqrMagnitude = pos2Light.x * pos2Light.x + pos2Light.y * pos2Light.y + pos2Light.y + pos2Light.z * pos2Light.z;

	auto m_emit = light.emit;
	auto m_eval = pos.m->eval(ray.direction, obj2LightDir, pos.normal);

    Ray obj2LightRay{ pos.coords, obj2LightDir };
    Intersection Light = intersect(obj2LightRay);

    if (Light.distance - distance >= -EPSILON)
    {
        L_dir = m_emit * m_eval * dotProduct(obj2LightDir, pos.normal) * dotProduct(-obj2LightDir, lightInter.normal) / sqrMagnitude / m_pdf;
    }

    if (get_random_float() >= RussianRoulette)
    {
        return L_dir;
    }

	// important samping for ggx
	if (pos.m->getType() == MicrofacetGlossy)
	{
		Vector3f wi = normalize(-ray.direction);
		// wo will be changed
		Vector3f wo{ 0.0f };

		float pdf_;

		Vector3f brdf = pos.m->ggxSample(wi, pos.normal, wo, pdf_);
		//printf("m_pdf = %f\n", m_pdf);

		if (pdf_ > 0)
		{
			wo = wo.normalized();
			Ray ref(pos.coords, wo);
			Intersection pos2 = intersect(ref);
			if (pos2.happened && !pos2.m->hasEmission()) {
				L_indir = castRay(ref, depth + 1) * brdf * fabsf(dotProduct(wo, pos.normal)) / (pdf_ * RussianRoulette);
				L_indir.x = clamp(0, L_indir.x, 1);
				L_indir.y = clamp(0, L_indir.y, 1);
				L_indir.z = clamp(0, L_indir.z, 1);
			}
		}
	}
	else
	{
		Vector3f objRSample = pos.m->sample(ray.direction, pos.normal);
		Ray obj2NextObjRay{ pos.coords, objRSample.normalized() };
		Intersection nextObjIntersection = intersect(obj2NextObjRay);

		if (nextObjIntersection.happened && !nextObjIntersection.m->hasEmission())
		{
			m_pdf = pos.m->pdf(ray.direction, objRSample.normalized(), pos.normal);
			Vector3f result = castRay(obj2NextObjRay, depth + 1);
			result.x = clamp(0, 1, result.x);
			result.y = clamp(0, 1, result.y);
			result.z = clamp(0, 1, result.z);
			L_indir = result * pos.m->eval(ray.direction, objRSample.normalized(), pos.normal)
				* dotProduct(objRSample.normalized(), pos.normal) / m_pdf / RussianRoulette;
		}
	}

    return L_dir + L_indir;
}