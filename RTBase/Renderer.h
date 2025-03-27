#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);

		// Sample a point on the light
		float lightPdf;
		Colour emitted;
		Vec3 p = light->sample(shadingData, sampler, emitted, lightPdf);
		Vec3 wi;
		float G = 0.0f;
		bool visible = false;

		if (light->isArea())
		{
			// Calculate GTerm
			wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			G = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;
			visible = G > 0 && scene->visible(shadingData.x, p);
		}
		else
		{
			// Calculate GTerm
			wi = p;
			float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			visible = G > 0 && scene->visible(shadingData.x, shadingData.x + wi * 10000.0f);
		}
		if (!visible)
			return Colour(0.0f, 0.0f, 0.0f);

		float bsdfPdf = shadingData.bsdf->PDF(shadingData, wi);
		float weight = (lightPdf + bsdfPdf > 0.0f) ? lightPdf / (lightPdf + bsdfPdf) : 0.0f;
		Colour f = shadingData.bsdf->evaluate(shadingData, wi);
		return f * emitted * G * weight / (pmf * lightPdf);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool canHitLight = true, Vec3 prevWi = Vec3(), float prevPdf = -1.0f)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true)
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			if (depth > MAX_DEPTH)
			{
				return direct;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}

			Colour sampledColour;
			float bsdfPdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, sampledColour, bsdfPdf);

			if (!shadingData.bsdf->isPureSpecular())
			{
				Colour bsdf = shadingData.bsdf->evaluate(shadingData, wi);
				pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / bsdfPdf;
			}
			else
			{
				pathThroughput = pathThroughput * sampledColour * fabsf(Dot(wi, shadingData.sNormal)) / bsdfPdf;
			}

			Ray nextRay(shadingData.x + (wi * EPSILON), wi);
			IntersectionData hit = scene->traverse(nextRay);
			ShadingData nextData = scene->calculateShadingData(hit, nextRay);

			if (hit.t < FLT_MAX && nextData.bsdf->isLight() && !shadingData.bsdf->isPureSpecular())
			{
				Colour Le = nextData.bsdf->emit(nextData, nextData.wo);
				float lightPdf = nextData.bsdf->PDF(nextData, -wi);
				float weight = (lightPdf + bsdfPdf > 0.0f) ? bsdfPdf / (lightPdf + bsdfPdf) : 0.0f;
				return direct + pathThroughput * Le * weight;
			}
			else if (hit.t >= FLT_MAX && scene->background && !shadingData.bsdf->isPureSpecular())
			{
				Colour Le = scene->background->evaluate(wi);
				float lightPdf = scene->background->PDF({}, wi);
				float weight = (lightPdf + bsdfPdf > 0.0f) ? bsdfPdf / (lightPdf + bsdfPdf) : 0.0f;
				return direct + pathThroughput * Le * weight;
			}

			return direct + pathTrace(nextRay, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular());
		}

		if (scene->background)
			return pathThroughput * scene->background->evaluate(r.dir);
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void render()
	{
		film->incrementSPP();

		const int tileSize = 16;

		int Nx = (film->width + tileSize - 1) / tileSize;
		int Ny = (film->height + tileSize - 1) / tileSize;
		int totalTiles = Nx * Ny;

		std::atomic<int> nextTileindex(0);

		// One thread per processor
		for (int i = 0; i < numProcs; i++) {
			threads[i] = new std::thread([&, i]()
				{
					// Loop each thread until no more tiles
					while (true) {
						// Grab the next tile index
						int tileIndex = nextTileindex.fetch_add(1);
						if (tileIndex >= totalTiles) break;

						// Convert that 1D tile index into (tileX, tileY)
						int tileX = tileIndex % Nx;
						int tileY = tileIndex / Nx;

						// Compute the pixel range for this tile
						int startX = tileX * tileSize;
						int startY = tileY * tileSize;
						int endX = min(startX + tileSize, (int)film->width);
						int endY = min(startY + tileSize, (int)film->height);

						// Get the random sampler for this thread
						MTRandom* sampler = &samplers[i];

						// Render all pixels in [startX,endX) x [startY,endY)
						for (int y = startY; y < endY; y++)
						{
							for (int x = startX; x < endX; x++)
							{
								//float px = x + 0.5f;
								//float py = y + 0.5f;
								float tx = x + sampler->next();
								float ty = y + sampler->next();
								Ray ray = scene->camera.generateRay(tx, ty);
								//Colour norm = viewNormals(ray);
								//Colour alb = albedo(ray);
								//Colour dir = direct(ray, sampler);
								Colour th(1.0f,1.0f,1.0f);
								Colour pathT = pathTrace(ray, th, 0, sampler);
								film->splat(tx, ty, pathT);
								unsigned char r, g, b;
								film->FilmicTonemap(x, y, r, g, b);
								canvas->draw(x, y, r, g, b);
							}
						}
						//canvas->present();

					}
				});
		}
		for (int i = 0; i < numProcs; i++)
		{
			threads[i]->join();
			delete threads[i];
		}
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};