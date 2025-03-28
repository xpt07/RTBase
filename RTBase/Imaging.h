#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) <= 0.5f && fabs(y) <= 0.5f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 0;
	}
};

class GaussianFilter : public ImageFilter
{
	const int radii = 1;
	const float alpha = 1.0f;
	const float t2 = std::exp(-alpha * SQ(radii));
public:
	float Gaussian(float d) const
	{
		return std::exp(-alpha * SQ(d)) - t2;
	}

	float filter(float x, float y) const
	{
		return Gaussian(x) * Gaussian(y);
	}

	int size() const
	{
		return radii;
	}
};

class Film
{
public:
	Colour* film;
	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;

	float A = 0.15;
	float B = 0.50;
	float C = 0.10;
	float D = 0.20;
	float E = 0.02;
	float F = 0.30;
	float W = 11.2;

	oidn::DeviceRef device;
	oidn::BufferRef colorBuffer, normalBuffer, albedoBuffer, outputBuffer;
	Vec3* colorData = nullptr;
	Vec3* normalData = nullptr;
	Vec3* albedoData = nullptr;
	Vec3* outputData = nullptr;

	void splat(const float x, const float y, const Colour& L) {
		float filterWeights[25]; // Storage to cache weights
		unsigned int indices[25]; // Store indices to minimize computations 
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(px - x, py - y);
					total += filterWeights[used];
					used++;
				}
			}
		}
		for (int i = 0; i < used; i++) {
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
		}
	}

	float Filmic(float x) {

		return ((x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F)) - (E / F);
	}

	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f)
	{
		Colour pixel = film[(y * width) + x] * exposure / (float)SPP;
		r = std::min(powf(std::max(pixel.r, 0.0f), 1.0f / 2.2f) * 255, 255.0f);
		g = std::min(powf(std::max(pixel.g, 0.0f), 1.0f / 2.2f) * 255, 255.0f);
		b = std::min(powf(std::max(pixel.b, 0.0f), 1.0f / 2.2f) * 255, 255.0f);
	}

	void FilmicTonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f)
	{
		Colour pixel = film[(y * width) + x] * exposure / (float)SPP;
		r = std::min(powf(std::max(Filmic(pixel.r) / Filmic(W), 0.0f), 1.0f / 2.2f) * 255, 255.0f);
		g = std::min(powf(std::max(Filmic(pixel.g) / Filmic(W), 0.0f), 1.0f / 2.2f) * 255, 255.0f);
		b = std::min(powf(std::max(Filmic(pixel.b) / Filmic(W), 0.0f), 1.0f / 2.2f) * 255, 255.0f);
	}

	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		filter = _filter;

		device = oidn::newDevice();
		device.commit();

		colorBuffer = device.newBuffer(width * height * sizeof(Vec3));
		normalBuffer = device.newBuffer(width * height * sizeof(Vec3));
		albedoBuffer = device.newBuffer(width * height * sizeof(Vec3));
		outputBuffer = device.newBuffer(width * height * sizeof(Vec3));

		colorData = (Vec3*)colorBuffer.getData();
		normalData = (Vec3*)normalBuffer.getData();
		albedoData = (Vec3*)albedoBuffer.getData();
		outputData = (Vec3*)outputBuffer.getData();

		clear();
	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));

		memset(colorData, 0, width * height * sizeof(Vec3));
		memset(normalData, 0, width * height * sizeof(Vec3));
		memset(albedoData, 0, width * height * sizeof(Vec3));
		memset(outputData, 0, width * height * sizeof(Vec3));

		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
	void recordAOVs(int x, int y, Colour colour, Vec3 normal, Colour albedo)
	{
		int idx = y * width + x;
		colorData[idx] = colorData[idx] + Vec3(colour.r, colour.g, colour.b);
		normalData[idx] = normalData[idx] + normal;
		albedoData[idx] = albedoData[idx] + Vec3(albedo.r, albedo.g, albedo.b);
	}

	void finalizeAOVs()
	{
		for (unsigned int i = 0; i < width * height; i++) {
			colorData[i] = colorData[i] / (float)SPP;
			normalData[i] = normalData[i] / (float)SPP;
			albedoData[i] = albedoData[i] / (float)SPP;
			outputData[i] = colorData[i];
		}
	}

	void saveDenoisedHDR(const std::string& filename)
	{
		oidn::FilterRef filter = device.newFilter("RT");
		filter.setImage("color", colorBuffer, oidn::Format::Float3, width, height);
		filter.setImage("normal", normalBuffer, oidn::Format::Float3, width, height);
		filter.setImage("albedo", albedoBuffer, oidn::Format::Float3, width, height);
		filter.setImage("output", outputBuffer, oidn::Format::Float3, width, height);
		filter.set("hdr", true);
		filter.commit();
		filter.execute();

		const char* errMsg;
		if (device.getError(errMsg) != oidn::Error::None)
			std::cerr << "OIDN ERROR: " << errMsg << std::endl;

		std::vector<float> pixels(width * height * 3);
		for (unsigned int i = 0; i < width * height; i++)
		{
			pixels[i * 3 + 0] = outputData[i].x / (float)SPP;
			pixels[i * 3 + 1] = outputData[i].y / (float)SPP;
			pixels[i * 3 + 2] = outputData[i].z / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, pixels.data());
	}
};