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

void Scene::sampleLight(Intersection& pos, double& pdf) const
{
	// 只对含自发光物体sample
	double emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	//double p = get_random_float() * emit_area_sum;
	double p = get_halton_random() * emit_area_sum;
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

	Intersection light;
	double m_pdf = 0.0;
	sampleLight(light, m_pdf);

	// Get x, ws, NN, emit from inter
	// Shoot a ray from p to x
	// 作业框架中符号说明
	// x 光源点
	// p 物体着色点
	// wo 往像素发射的入射光线
	// ws 物体往光源发射的射线
	// N objectInter法线向量
	// NN lightInter法线向量

	// 本文中符号说明
	// wo eye_pos往交点的入射光线
	// wi 交点往光源点的射线

	// pos
	Vector3f N = pos.normal;
	Vector3f NN = light.normal;

	Vector3f wo = normalize(-ray.direction);
	Vector3f pos2Light = light.coords - pos.coords;
	Vector3f ws = normalize(pos2Light);

	float distance = pos2Light.norm();
    float sqrMagnitude = pos2Light.x * pos2Light.x + pos2Light.y * pos2Light.y + pos2Light.z * pos2Light.z;

	Ray pos2LightRay{ pos.coords, ws };
    Intersection lightInter = intersect(pos2LightRay);

	// 判断交点与光源点之间是否有其他物体遮挡
    if (lightInter.happened && lightInter.distance - distance >= -EPSILON)
    {
		// 此时看成ws射向交点，从wo反射回eys_pos
        L_dir = light.emit * pos.m->eval(ws, wo, N)
			* dotProduct(ws, N) * dotProduct(-ws, NN) / sqrMagnitude / m_pdf;

		L_dir.x = clamp(0, L_dir.x, 1);
		L_dir.y = clamp(0, L_dir.y, 1);
		L_dir.z = clamp(0, L_dir.z, 1);
    }

    //if (get_random_float() >= RussianRoulette)
    if (get_halton_random() >= RussianRoulette)
    {
        return L_dir;
    }

	Vector3f L_indir = { 0,0,0 };

	// important samping for ggx
	if (pos.m->getType() == MicrofacetGlossy)
	{
		// ggx特殊采样出wi方向向量
		Vector3f wi;
		float pdf_;

		Vector3f brdf = pos.m->ggxSample(wo, N, wi, pdf_);

		if (pdf_ > 0)
		{
			wi = wi.normalized();
			Ray ref(pos.coords, wi);
			Intersection pos2 = intersect(ref);
			if (pos2.happened && !pos2.m->hasEmission()) {
				Vector3f result = castRay(ref, depth + 1);
				result.x = clamp(0, result.x, 1);
				result.y = clamp(0, result.y, 1);
				result.z = clamp(0, result.z, 1);
				L_indir = result * brdf * fabsf(dotProduct(wi, N)) / pdf_ / RussianRoulette;
				//format(L_indir);
			}
		}
	}
	else
	{
		// 采样出wi方向向量
		Vector3f wi = normalize(pos.m->sample(wo, N));
		Ray pos2NextObjRay{ pos.coords, wi };
		Intersection nextObjIntersection = intersect(pos2NextObjRay);

		if (nextObjIntersection.happened && !nextObjIntersection.m->hasEmission())
		{
			//m_pdf = pos.m->pdf(ray.direction, wi, pos.normal);
			Vector3f result = castRay(pos2NextObjRay, depth + 1);
			result.x = clamp(0, 1, result.x);
			result.y = clamp(0, 1, result.y);
			result.z = clamp(0, 1, result.z);
			L_indir = result * pos.m->eval(wo, wi, N)
				* dotProduct(wi, N) / pos.m->pdf(wo, wi, N) / RussianRoulette;

			//format(L_indir);
		}
	}

    return L_dir + L_indir;
}