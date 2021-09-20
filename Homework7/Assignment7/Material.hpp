//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, Microfacet, MicrofacetGlossy};

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    // \param N is the normal at the intersection point
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

	Vector3f fresnelSchilck(const Vector3f& normal, const Vector3f& View2Point, const Vector3f& F0)
	{
		Vector3f one = Vector3f(1.0f);
		return F0 + (one - F0) * powf(1.0 - fmaxf(dotProduct(normal, View2Point), 0.0f), 5.0);
	}

	float GeometryFunction(const Vector3f& normal, const Vector3f& view2Point, const Vector3f& lightDir, const float& roughness) const
	{
		// GGX Schilick-Beckmann
		// include Geometry Obstruction and Geometry Shadowing
		// Smith's Method

		// G(n, v, l, k) = Gsub(n, v, k)Gsub(n, l, k)
		// kdirect = (a + 1) * (a + 1) / 8
		float k = pow(roughness + 1.0f, 2.0f) / 8.0f;

		float ggx1 = GeometryFunctionCalculate(normal, view2Point, k);
		float ggx2 = GeometryFunctionCalculate(normal, lightDir, k);

		return ggx1 * ggx2;
	}

	float GeometryFunctionCalculate(const Vector3f& normal, const Vector3f& temp, float k) const
	{
		float nDotTemp = fmaxf(dotProduct(normal, temp), 0.0f);
		float nDotTempDotK = nDotTemp * (1.0f - k) + k;
		return nDotTemp / nDotTempDotK;
	}

	float NormalDistributionFunction(const Vector3f& normal, const Vector3f& half_vector, const float& roughness) const
	{
		// set : x = (n · h) * (n · h) * (a * a - 1) + 1
        // NDF = a * a / (M_PI * x * x)
        float a2 = roughness * roughness;
        float nDotH2 = pow(fmaxf(dotProduct(normal, half_vector), 0.0f), 2.0f);
        float denom = M_PI * pow(nDotH2 * (a2 - 1.0f) + 1.0f, 2);
        return a2 / denom;
	}

	Vector3f toWorld(const Vector3f& a, const Vector3f& N) {
		Vector3f B, C;
		if (std::fabs(N.x) > std::fabs(N.y)) {
			float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
			C = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
		}
		else {
			float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
			C = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
		}
		B = crossProduct(C, N);
		return a.x * B + a.y * C + a.z * N;
	}

public:
	MaterialType m_type;
	//Vector3f m_color;
	Vector3f m_emission;
	//float roughness;
	float ior;
	Vector3f Kd, Ks;
	float specularExponent;
	//Texture tex;

	inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
	inline MaterialType getType();
	//inline Vector3f getColor();
	inline Vector3f getColorAt(double u, double v);
	inline Vector3f getEmission();
	inline bool hasEmission();

	// sample a ray by Material properties
	inline Vector3f sample(const Vector3f& wi, const Vector3f& N);
	// given a ray, calculate the PdF of this ray
	inline float pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
	// given a ray, calculate the contribution of this ray
	inline Vector3f eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N);
};

Material::Material(MaterialType t, Vector3f e) {
	m_type = t;
	//m_color = c;
	m_emission = e;
}

MaterialType Material::getType() { return m_type; }
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() { return m_emission; }
bool Material::hasEmission() {
	if (m_emission.norm() > EPSILON) return true;
	else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}

/// <summary>
/// 根据法线，采样一个出射光线，并转换到世界坐标系下
/// </summary>
/// <param name="wi"></param>
/// <param name="N"></param>
/// <returns></returns>
Vector3f Material::sample(const Vector3f& wi, const Vector3f& N)
{
	switch (m_type)
	{
		// 对漫反射材质采样与入射光线无关
	case DIFFUSE: case Microfacet: case MicrofacetGlossy:
	{
		// uniform sample on the hemisphere
		float x_1 = get_random_float(), x_2 = get_random_float();
		// z belongs to [-1, 1]
		float z = std::fabs(1.0f - 2.0f * x_1);
		// r belongs to [0, 1]
		// phi 半球的立体角是2pi
		float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
		// 均匀平滑r的分布 [0, 1]
		// 采样大小和方向
		Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);

		//float r_1 = get_random_float(), r_2 = get_random_float();
		//float x = std::cos(2 * M_PI * r_1) * 2 * std::sqrt(r_2 * (1 - r_2));
		//float y = std::sin(2 * M_PI * r_1) * 2 * std::sqrt(r_2 * (1 - r_2));
		//float z = 1 - 2 * r_2;

		//Vector3f localRay(x, y, z);
		return toWorld(localRay, N);

		break;
	}
	}
}

float Material::pdf(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) {
	switch (m_type) {
	case DIFFUSE: case Microfacet: case MicrofacetGlossy:
	{
		// uniform sample probability 1 / (2 * PI)
		if (dotProduct(wo, N) > -EPSILON)
			return 0.5f / M_PI;
		else
			return 0.0f;
		break;
	}
	}
}

/// <param name="wi">入射</param>
/// <param name="wo">出射</param>
/// <param name="N">法向量</param>
/// <returns>return diffuse; Vector3f diffuse = Kd / M_PI;</returns>
Vector3f Material::eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) {
	switch (m_type) {
	case DIFFUSE:
	{
		// calculate the contribution of diffuse   model
		float cosalpha = dotProduct(N, wo);
		if (cosalpha > -EPSILON) {
			Vector3f diffuse = Kd / M_PI;
			return diffuse;
		}
		else
			return Vector3f(0.0f);
		break;
	}
	case Microfacet:
	{
		float cosalpha = dotProduct(N, wo);
		if (cosalpha > -EPSILON) {
			float roughness = 0.001f;
			//float refractive = 1.85f;

			Vector3f View2Point = -wi;
			Vector3f lightDir = wo;

			Vector3f half_vector = (View2Point + lightDir).normalized();

			float D = NormalDistributionFunction(N, half_vector, roughness);
			float F;
			float G = GeometryFunction(N, View2Point, lightDir, roughness); 

			// Tips:wi
			fresnel(wi, N, ior, F);
			//Vector3f F0(0.95f, 0.93f, 0.88f);
			//Vector3f vec_fresnel = fresnelSchilck(N, View2Point, F0);

			float crossWiWo = 4 * fmaxf(dotProduct(View2Point, N), 0.0f) * fmaxf(dotProduct(lightDir, N), 0.0f);

			float f_diffuse = 1.0f / M_PI;
			float f_cook_torrance = D * F * G / (std::max(crossWiWo, 0.001f));

			// enery conservation
			//Vector3f Ks = Vector3f(F);
			Vector3f _Ks = Vector3f(1.0f) - Kd;
			//Vector3f KD = Vector3f(1.0f) - vec_fresnel;

			return (1 - F) * Kd * f_diffuse + F * f_cook_torrance;
		}
		else
			return Vector3f(0.0f);
		break;
	}
	case MicrofacetGlossy:
	{
		float cosalpha = dotProduct(N, wo);
		if (cosalpha > -EPSILON) {
			// Blinn-Phong
			Vector3f View2Point = -wo;
			Vector3f lightDir = wi;

			Vector3f half_vector = (View2Point + lightDir).normalized();
			//float nDotHalf = fmaxf(dotProduct(N, half_vector), 0.0f);

			//float roughness = 0.01f;
			//float F;
			//fresnel(wi, N, ior, F);
			//float G = GeometryFunction(N, View2Point, lightDir, roughness);
			//float weight_inv = fabs((fmaxf(dotProduct(View2Point, N), 0.0f) * fmaxf(dotProduct(half_vector, N), 0.0f))) / fmaxf(dotProduct(View2Point, half_vector), 0.0f);

			//return G * weight_inv * F;

			//auto h = normalize(wi + wo);
			double p = 25;
			double spec = pow(std::max(0.0f, dotProduct(N, half_vector)), p);
			auto ans = Ks * spec + Kd / M_PI;
			//  clamp(0, 1, ans.x); clamp(0, 1, ans.y); clamp(0, 1, ans.z);
			return ans;
		}
		else
			return Vector3f(0.0f);
		break;
	}
	}
}

#endif //RAYTRACING_MATERIAL_H
