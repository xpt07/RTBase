#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class TabulatedDistribution
{
public:
	unsigned int width = 0;
	unsigned int height = 0;

	std::vector<float> luminanceMap;
	std::vector<float> marginalCDF;
	std::vector<std::vector<float>> conditionalCDF;

	void clear()
	{
		luminanceMap.clear();
		marginalCDF.clear();
		conditionalCDF.clear();
	}

	void init(Texture* texture)
	{
		clear();

		width = texture->width;
		height = texture->height;

		luminanceMap.resize(width * height);
		marginalCDF.resize(height);
		conditionalCDF.resize(height, std::vector<float>(width));

		float total = 0.0f;

		for (unsigned int y = 0; y < height; ++y)
		{
			float sinWeight = sinf(((float)y + 0.5f) / height * M_PI);
			float rowSum = 0.0f;

			for (unsigned int x = 0; x < width; ++x)
			{
				unsigned int idx = y * width + x;
				luminanceMap[idx] = texture->texels[idx].Lum() * sinWeight;
				rowSum += luminanceMap[idx];
				conditionalCDF[y][x] = rowSum;
			}

			marginalCDF[y] = rowSum;
			total += rowSum;
		}

		for (auto& row : conditionalCDF)
		{
			float rowTotal = row.back();
			if (rowTotal > 0.0f)
			{
				for (auto& val : row)
					val /= rowTotal;
				row.back() = 1.0f;
			}
		}

		if (total > 0.0f)
		{
			float invTotal = 1.0f / total;
			float cumulative = 0.0f;
			for (auto& val : marginalCDF)
			{
				cumulative += val * invTotal;
				val = cumulative;
			}
			marginalCDF.back() = 1.0f;
		}
	}

	static int binarySearch(const std::vector<float>& cdf, int size, float value)
	{
		int low = 0, high = size - 1;
		while (low < high)
		{
			int mid = (low + high) / 2;
			if (cdf[mid] < value)
				low = mid + 1;
			else
				high = mid;
		}
		return low;
	}

	float getPdf(int row, int col)
	{
		float marginal = (row == 0) ? marginalCDF[row] : (marginalCDF[row] - marginalCDF[row - 1]);
		float conditional = (col == 0) ? conditionalCDF[row][col] : (conditionalCDF[row][col] - conditionalCDF[row][col - 1]);
		float result = marginal * conditional * width * height;
		return (result < EPSILON) ? EPSILON : result;
	}

	Vec3 sample(Sampler* sampler, float& u, float& v, float& pdf)
	{
		int row = binarySearch(marginalCDF, height, sampler->next());
		int col = binarySearch(conditionalCDF[row], width, sampler->next());

		u = (col + 0.5f) / width;
		v = (row + 0.5f) / height;

		pdf = getPdf(row, col);

		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;

		pdf *= sinf(theta);

		return Vec3(
			sinf(theta) * cosf(phi),
			cosf(theta),
			sinf(theta) * sinf(phi)
		);
	}
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		Vec3 wi = Vec3(0, 0, 1);
		pdf = 1.0f;
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	TabulatedDistribution distribution;
	EnvironmentMap(Texture* _env)
	{
		env = _env;
		distribution.init(env);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		float u, v;
		Vec3 wi = distribution.sample(sampler, u, v, pdf);
		reflectedColour = env->sample(u, v);
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		if (u < 0.0f) u += 2.0f * M_PI;
		u /= (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		
		int row = std::min(int(v * distribution.height), (int)distribution.height - 1);
		int col = std::min(int(u * distribution.width), (int)distribution.width - 1);

		return distribution.getPdf(row, col);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		float u, v;
		return distribution.sample(sampler, u, v, pdf);
	}
};