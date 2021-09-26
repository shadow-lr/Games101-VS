//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, Microfacet, MicrofacetGlossy, MICROFACET_Test1, MICROFACET_Test2};

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
		// if parme is float ior
		// F0 = pow((ior - 1) / (ior + 1), 2)
		return F0 + (one - F0) * powf(1.0 - dotProduct(normal, View2Point), 5.0);
	}

	float GeometrySmith(const Vector3f& normal, const Vector3f& V, const Vector3f& L, float k) const
	{
		float NdotV = fmaxf(dotProduct(normal, V), 0.0f);
		float NdotL = fmaxf(dotProduct(normal, L), 0.0f);
		float ggx1 = GeometrySchlickGGX(NdotV, k);
		float ggx2 = GeometrySchlickGGX(NdotL, k);

		return ggx1 * ggx2;
	}

	float GeometrySchlickGGX(float NdotV, float k) const
	{
		float nom = NdotV;
		float denom = NdotV * (1.0 - k) + k;
		
		return nom / denom;
	}

	float GeometryFunction(const Vector3f& normal, const Vector3f& view2Point, const Vector3f& lightDir, const float& roughness) const
	{
		// GGX Schilick-Beckmann
		// include Geometry Obstruction and Geometry Shadowing
		// Smith's Method

		// G(n, v, l, k) = Gsub(n, v, k)Gsub(n, l, k)
		// kdirect = (a + 1) * (a + 1) / 8
		//float k = pow(roughness + 1.0f, 2.0f) / 8.0f;
		float k = roughness;

		float ggx1 = GeometryFunctionCalculate(normal, view2Point, k);
		float ggx2 = GeometryFunctionCalculate(normal, lightDir, k);

		return ggx1 * ggx2;
	}

	float GeometryFunctionCalculate(const Vector3f& normal, const Vector3f& temp, float k) const
	{
		double nDotTemp = fmaxf(dotProduct(normal, temp), 0.0f);
		double nDotTempDotK = nDotTemp * (1.0f - k) + k;
		return nDotTemp / nDotTempDotK;
	}

	float NormalDistributionFunction(const Vector3f& normal, const Vector3f& half_vector, const float& roughness) const
	{
		// set : x = (n · h) * (n · h) * (a * a - 1) + 1
        // NDF = a * a / (M_PI * x * x)
		float a2 = roughness * roughness;
		float nDotH = fmaxf(dotProduct(normal, half_vector), 0.0f);
		float nDotH2 = nDotH * nDotH;

		float nom = a2;
		// 此处float存会溢出 denom 会趋向于0 导致结果接近无穷大
		double denom = nDotH2 * (a2 - 1.0) + 1.0;
        //float denom = M_PI * powf(nDotH2 * (a2 - 1.0f) + 1.0f, 2.0f);
		denom = M_PI * pow(denom, 2.0);
        return nom / denom;
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
	float roughness;
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

	inline float fresnelSchilick(const Vector3f& wi, const Vector3f& half_vector, const float& ior);

	// specular DDX important sample in ND
	inline Vector3f SchlickFresnel(const Vector3f& r0, float radians);
	inline float SmithGGXMasking(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, float a2);
	inline float SmithGGXMaskingShadowing(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, float a2);
	inline Vector3f SphericalToCartesian(const float theta, const float phi);
	inline void ImportanceSampleGgxD(Vector3f& wi, const Vector3f& wo, const Vector3f& N, Vector3f& reflectance);
	inline Vector3f GgxVndf(const Vector3f& wo, float roughness, float u1, float u2);
	inline void ImportanceSampleGgxVdn(Vector3f& wg, Vector3f& wo, const Vector3f& N, Vector3f& wi, Vector3f& reflectance);

	/// <param name="wi">入射光线</param>
	/// <param name="N">法线</param>
	/// <param name="wo">出射光先</param>
	/// <param name="pdf">概率密度函数</param>
	/// <returns></returns>
	inline Vector3f ggxSample(Vector3f& wi, const Vector3f& N, Vector3f& wo, float& pdf);
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

inline float Material::fresnelSchilick(const Vector3f& wi, const Vector3f& half_vector, const float& ior)
{
	double cosTheta = dotProduct(wi, half_vector);
	double R0 = powf((ior - 1) / (ior + 1), 2);
	return R0 + (1 - R0) * powf((1 - cosTheta), 5);
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
	case DIFFUSE:case Microfacet:case MicrofacetGlossy:
	{
		// uniform sample on the hemisphere
		//float x_1 = get_random_float(), x_2 = get_random_float();
		auto random_uv = get_halton_random_pair();
		auto x_1 = random_uv.first, x_2 = random_uv.second;

		double z = std::fabs(1.0f - 2.0f * x_1);
		double r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
		Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
		return toWorld(localRay, N);

		break;
	}
	//case MicrofacetGlossy:
	//{
	//	return toWorld(reflect(-wi, N), N);
	//}
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

/// <param name="wi">pos2light</param>
/// <param name="wo">-ray.direction()</param>
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
			//float roughness = 0.1f;
			//float refractive = 1.85f;

			Vector3f half_vector = (wo + wi).normalized();

			float D = NormalDistributionFunction(N, half_vector, roughness);
			float F;
			float G = GeometryFunction(N, wo, wi, roughness); 

			// Tips:wi
			fresnel(wi, N, ior, F);
			//Vector3f F0(0.95f, 0.93f, 0.88f);
			//Vector3f vec_fresnel = fresnelSchilck(N, View2Point, F0);

			float crossWiWo = 4 * fmaxf(dotProduct(wi, N), 0.0f) * fmaxf(dotProduct(wo, N), 0.0f);

			float f_diffuse = 1.0f / M_PI;
			float f_cook_torrance = D * F * G / (std::max(crossWiWo, 0.001f));

			// enery conservation
			//Vector3f Ks = Vector3f(F);
			 Vector3f Ks = Vector3f(1.0f) - Kd;
			//Vector3f _Ks = Vector3f(1.0f) - Kd;
			//Vector3f KD = Vector3f(1.0f) - vec_fresnel;

			return Kd * f_diffuse + F * f_cook_torrance;
		}
		else
			return Vector3f(0.0f);
		break;
	}
	case MicrofacetGlossy:
	{
		float cosalpha = dotProduct(N, wo);
		float cosbeta = dotProduct(N, wi);

		if (cosalpha * cosbeta > -EPSILON) {
			Vector3f h = normalize(wi + wo);
			//Vector3f fr = fresnelSchilck(N, h, { 0.03f,0.03,0.03f });
			float fr = fresnelSchilick(wo, h, ior);
			float D = NormalDistributionFunction(N, h, roughness);
			//float G = GeometryFunction(N, Point2View, wo, roughness);
			float G = GeometrySmith(N, wi, wo, roughness);

			float bsdf = fr * D * G / fabsf(4 * cosbeta * cosbeta);

			return Kd * bsdf;
		}
		else
			return Vector3f(0);
		break;


		//if (cosalpha > -EPSILON) {

		//	Vector3f sample_wi{ 0 };
		//	Vector3f reflectance{ 0 };
		//	Vector3f wg{ 0 };

		//	Vector3f this_wo = -wi;
		//	Vector3f this_wi = { 0 };

		//	ImportanceSampleGgxD(sample_wi, wo, N, reflectance);
		//	//ImportanceSampleGgxVdn(wg, this_wo, N, this_wi, reflectance);

		//	return Kd / M_PI + reflectance;

		//	//float roughness = 0.1f;
		//	//float refractive = 1.85f;

		//	Vector3f View2Point = -wi;
		//	Vector3f lightDir = wo;

		//	Vector3f half_vector = (View2Point + lightDir).normalized();

		//	float D = NormalDistributionFunction(N, half_vector, roughness);
		//	float F;
		//	float G = GeometryFunction(N, View2Point, lightDir, roughness);

		//	// Tips:wi
		//	fresnel(wi, N, ior, F);
		//	//Vector3f F0(0.95f, 0.93f, 0.88f);
		//	//Vector3f vec_fresnel = fresnelSchilck(N, View2Point, F0);

		//	float crossWiWo = 4 * fmaxf(dotProduct(View2Point, N), 0.0f) * fmaxf(dotProduct(lightDir, N), 0.0f);

		//	float f_diffuse = 1.0f / M_PI;
		//	float f_cook_torrance = D * F * G / (std::max(crossWiWo, 0.001f));

		//	// enery conservation
		//	//Vector3f Ks = Vector3f(F);
		//	Vector3f Ks = Vector3f(1.0f) - Kd;
		//	//Vector3f _Ks = Vector3f(1.0f) - Kd;
		//	//Vector3f KD = Vector3f(1.0f) - vec_fresnel;

		//	return Kd * f_diffuse + F * f_cook_torrance;
		//}
		//else
		//	return Vector3f(0.0f);
		//break;
	}
	}
}

Vector3f Material::SchlickFresnel(const Vector3f& r0, float radians)
{
	// -- The common Schlick Fresnel approximation
	float exponential = powf(1.0f - radians, 5.0f);
	// 假设此处是玻璃
	//Vector3f r0 = Vector3f(0.5, 0.5, 0.5);
	return r0 + (Vector3f(1.0f) - r0) * exponential;
}

float Material::SmithGGXMasking(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, float a2)
{
	float dotNL = dotProduct(N, wi);
	float dotNV = dotProduct(N, wo);
	float denomC = sqrtf(a2 + (1.0f - a2) * dotNV * dotNV) + dotNV;

	return 2.0f * dotNV / denomC;
}

//====================================================================
// non height-correlated masking-shadowing function is described here:
float Material::SmithGGXMaskingShadowing(const Vector3f& wi, const Vector3f& wo, const Vector3f& N, float a2)
{
	float dotNL = dotProduct(N, wi);
	float dotNV = dotProduct(N, wo);

	float denomA = dotNV * sqrtf(a2 + (1.0f - a2) * dotNL * dotNL);
	float denomB = dotNL * sqrtf(a2 + (1.0f - a2) * dotNV * dotNV);

	return 2.0f * dotNL * dotNV / (denomA + denomB);
}

Vector3f Material::SphericalToCartesian(const float theta, const float phi)
{
	//float x = std::cos(theta);
	//float y = std::sin(theta) * std::cos(phi);
	//float z = std::sin(theta) * std::sin(phi);

	float x = std::sin(theta) * std::cos(phi);
	float y = std::sin(theta) * std::sin(phi);
	float z = std::cos(theta);

	// y轴
	//float x = std::sin(theta) * std::cos(phi);
	//float y = std::cos(theta);
	//float z = std::sin(theta) * std::sin(phi);

	return Vector3f(x, y, z);
}

Vector3f Material::GgxVndf(const Vector3f& wo, float roughness, float u1, float u2)
{
	// -- Stretch the view vector so we are sampling as though
	// -- roughness==1
	Vector3f v = Vector3f(wo.x * roughness,
		wo.y,
		wo.z * roughness).normalized();

	// -- Build an orthonormal basis with v, t1, and t2
	Vector3f t1 = (v.y < 0.999f) ? crossProduct(v, Vector3f(0, 0, 1)) : Vector3f(1, 0, 0);
	Vector3f t2 = crossProduct(t1, v);

	// -- Choose a point on a disk with each half of the disk weighted
	// -- proportionally to its projection onto direction v
	float a = 1.0f / (1.0f + v.y);
	float r = sqrtf(u1);
	float phi = (u2 < a) ? (u2 / a) * M_PI : M_PI + (u2 - a) / (1.0f - a) * M_PI;
	float p1 = r * std::cos(phi);
	float p2 = r * std::sin(phi) * ((u2 < a) ? 1.0f : v.y);

	// -- Calculate the normal in this stretched tangent space
	Vector3f n = p1 * t1 + p2 * t2 + sqrtf(std::fmaxf(0.0f, 1.0f - p1 * p1 - p2 * p2)) * v;

	// -- unstretch and normalize the normal
	return Vector3f(roughness * n.x, std::max(0.0f, n.y), roughness * n.z).normalized();
}


/// <summary>
/// 
/// </summary>
/// <param name="wg">法线</param>
/// <param name="wo"></param>
/// <param name="wi"></param>
/// <param name="N"></param>
/// <param name="reflectance"></param>
void Material::ImportanceSampleGgxD(Vector3f& wi, const Vector3f& wo, const Vector3f& N, Vector3f& reflectance)
{
	float a = roughness;
	float a2 = a * a;

	// -- Generate uniform random variables between 0 and 1
	float e0 = get_random_float();
	float e1 = get_random_float();

	// -- Calculate theta and phi for our microfacet normal wm by
	// -- importance sampling the Ggx distribution of normals
	float theta = acosf(sqrtf((1.0f - e0) / ((a2 - 1.0f) * e0 + 1.0f)));
	float phi = 2 * M_PI * e1;

	// -- Convert from spherical to Cartesian coordinates
	Vector3f wm = SphericalToCartesian(theta, phi);
	wm = toWorld(wm, N);

	// -- Calculate wi by reflecting wo about wm
	wi = 2.0f * dotProduct(wo, wm) * wm - (wo);

	// -- Ensure our sample is in the upper hemisphere
	// -- Since we are in tangent space with a y-up coordinate
	// -- system BsdfNDot(wi) simply returns wi.y
	if (dotProduct(N, wi) > -EPSILON && dotProduct(wi, wm) > -EPSILON) {

		//float dotWiWm = fmaxf(dotProduct(wi, wm), 0.01f);
		float F;

		// -- calculate the reflectance to multiply by the energy
		// -- retrieved in direction wi
		fresnel(-wi, N, 20.0f, F);
		//Vector3f F = SchlickFresnel({ 0.08f,0.08f,0.08f }, dotWiWm);
		float G = SmithGGXMaskingShadowing(wi, wo, N, a2);
		float weight = fabsf(dotProduct(wo, wm)) / (dotProduct(N, wo) * dotProduct(N, wm));

		reflectance = Vector3f(F * G * weight);
	}
}

void Material::ImportanceSampleGgxVdn(Vector3f& wg, Vector3f& wo, const Vector3f& N, Vector3f& wi, Vector3f& reflectance)
{
	//Vector3f specularColor = material->specularColor;
	Vector3f specularColor = Vector3f(0.5f, 0.5f, 0.5f);
	float a = roughness;
	float a2 = a * a;

	float r0 = get_random_float();
	float r1 = get_random_float();
	Vector3f wm = GgxVndf(wo, roughness, r0, r1);

	wi = reflect(wm, wo);
	wi = toWorld(wi, N);

	// BsdfNDot(wi) > 0.0f
	if (dotProduct(N, wi) > -EPSILON) {

		Vector3f F = SchlickFresnel(specularColor, fmaxf(dotProduct(wi, wm), 0.0f));
		float G1 = SmithGGXMasking(wi, wo, N, a2);
		float G2 = SmithGGXMaskingShadowing(wi, wo, N, a2);

		reflectance = F * (G2 / G1);

		std::cout << "F.x = " << F.x << "\t" << "F.y = " << F.y << "\t" << "F.z = " << F.z << std::endl;
	}
	else {
		reflectance = Vector3f(0.0f);
	}
}

/// <param name="wi">-ray.direction()</param>
/// <param name="N">法线向量</param>
/// <param name="wo">需要采样出的出射光线</param>
/// <param name="pdf">概率密度函数</param>
/// <returns></returns>
Vector3f Material::ggxSample(Vector3f& wi, const Vector3f& N, Vector3f& wo, float& pdf) {
	double a = roughness;
	double a2 = a * a;

	auto random_e01 = get_halton_random_pair();
	double e0 = random_e01.first;
	double e1 = random_e01.second;
	//double e0 = get_random_float();
	//double e1 = get_random_float();
	double cos2Theta = (1 - e0) / (e0 * (a2 - 1) + 1);
	double cosTheta = sqrt(cos2Theta);
	double sinTheta = sqrt(1 - cos2Theta);
	double phi = 2 * M_PI * e1;
	Vector3f localdir(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);

	// sample half vector
	Vector3f h = toWorld(localdir, N);

	double fr = fresnelSchilick(wi, h, ior);
	//bool isReflect = get_random_float() <= fr;
	bool isReflect = get_halton_random() <= fr;
	if (isReflect) {
		wo = h * 2.0f * dotProduct(wi, h) - wi;
		if (dotProduct(wi, N) * dotProduct(wo, N) <= 0) {
			pdf = 0;
			return Vector3f(0);
		}

		double D = NormalDistributionFunction(N, h, roughness);
		pdf = fr * D * dotProduct(h, N) / (4 * (fabsf(dotProduct(wi, h))));
		double G = GeometryFunction(N, wo, wi, roughness);
		double bsdf = fr * D * G / fabsf(4.0 * dotProduct(N, wo) * dotProduct(N, wi));

		return Kd * bsdf;
	}
	else {
		pdf = 0;
		return Vector3f(0);
	}
}

#endif //RAYTRACING_MATERIAL_H
