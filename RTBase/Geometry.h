#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		t = (d - n.dot(r.o)) / n.dot(r.dir);
		return t >= 0;
	}
};

#define EPSILON 1e-7f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	Vec3 maxP, minP;
	Vec3 center;
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;

		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;

		e1 = vertices[0].p - vertices[2].p;
		e2 = vertices[1].p - vertices[2].p;

		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);

		maxP = Max(vertices[0].p, Max(vertices[1].p, vertices[2].p));
		minP = Min(vertices[0].p, Min(vertices[1].p, vertices[2].p));

		center = minP + (maxP - minP) * 0.5f;
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}

	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 p = Cross(r.dir, e2);
		float det = p.dot(e1);

		if (std::abs(det) < EPSILON)
			return false;

		float invDet = 1.0f / det;
		Vec3 T = r.o - vertices[2].p;

		u = T.dot(p) * invDet;

		if ((u < 0 && abs(u) > EPSILON) || (u > 1 && abs(u - 1) > EPSILON))
			return false;

		p = Cross(T, e1);
		v = r.dir.dot(p) * invDet;

		if ((v < 0 && abs(v) > EPSILON) || (u + v > 1 && abs(u + v - 1) > EPSILON))
			return false;

		t = e2.dot(p) * invDet;

		if (t < EPSILON)
			return false;

		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();
		
		float alpha = 1 - sqrt(r1);
		float beta = r2 * sqrt(r1);
		float gamma = 1 - (alpha + beta);
		
		pdf = 1 / area;
		
		Vec3 p = vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;

		return p;
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	Vec3 center;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
		updateCenter();
	}

	void updateCenter()
	{
		center = (min + max) * 0.5f;
	}

	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
		updateCenter();
	}
	void extend(const AABB& other)
	{
		extend(other.min);
		extend(other.max);
	}
	void extend(Triangle triangle) {
		extend(triangle.vertices[0].p);
		extend(triangle.vertices[1].p);
		extend(triangle.vertices[2].p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 tMin = (min - r.o) * r.invDir;
		Vec3 tMax = (max - r.o) * r.invDir;

		Vec3 tEnter = Min(tMin, tMax);
		Vec3 tExit = Max(tMin, tMax);

		float t_entry = std::max(tEnter.x, std::max(tEnter.y, tEnter.z));
		float t_exit = std::min(tExit.x, std::min(tExit.y, tExit.z));

		if (t_entry > t_exit || t_exit < 0) return false; // No intersection
		t = (t_entry < 0) ? t_exit : t_entry; // If inside the box, use t_exit
		return true;
	}

	bool rayAABB(const Ray& r)
	{
		float t;
		return rayAABB(r, t);
	}

	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32
#define MAX_DEPTH 16 

class BVHNode
{
public:
	AABB bounds;
	BVHNode* right;
	BVHNode* left;
	std::vector<int> triangleIndices; // For leaf nodes

#ifdef DEBUG_BVH
	static int totalLeafNodes;
	static int totalInternalNodes;
#endif

	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		right = NULL;
		left = NULL;
	}

	~BVHNode()
	{
		delete left;
		delete right;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& triangles, std::vector<int>& indices, int depth = 0)
	{
#ifdef DEBUG_BVH
		if (depth == 0) {
			totalLeafNodes = 0;
			totalInternalNodes = 0;
			std::cout << "[BVH] Building BVH..." << std::endl;
		}
#endif

		// Compute bounding box
		bounds.reset();
		for (int i : indices)
			bounds.extend(triangles[i]);

		// Stop if max depth is reached or few triangles remain
		if (indices.size() <= MAXNODE_TRIANGLES || depth >= MAX_DEPTH)
		{
			triangleIndices = indices;

#ifdef DEBUG_BVH
			totalLeafNodes++;
			std::cout << "[BVH] Leaf Node | Depth: " << depth
				<< " | Triangles: " << triangleIndices.size()
				<< " | Total Leaves: " << totalLeafNodes << std::endl;
#endif
			return;
		}

		// Compute the longest axis
		Vec3 extent = bounds.max - bounds.min;
		int axis = (extent.x > extent.y && extent.x > extent.z) ? 0 : (extent.y > extent.z) ? 1 : 2;

		// Sort triangles along the chosen axis
		std::sort(indices.begin(), indices.end(), [&](int a, int b) {
			return triangles[a].centre()[axis] < triangles[b].centre()[axis];
			});

		// Split at the median
		size_t mid = indices.size() / 2;
		std::vector<int> leftIndices(indices.begin(), indices.begin() + mid);
		std::vector<int> rightIndices(indices.begin() + mid, indices.end());

		if (leftIndices.empty() || rightIndices.empty()) {
			// If split failed, force leaf node
			triangleIndices = indices;
#ifdef DEBUG_BVH
			std::cout << "[BVH] Split Failed - Creating Leaf | Depth: " << depth
				<< " | Triangles: " << triangleIndices.size() << std::endl;
			totalLeafNodes++;
#endif
			return;
		}

		// Create child nodes
		left = new BVHNode();
		right = new BVHNode();
		left->build(triangles, leftIndices, depth + 1);
		right->build(triangles, rightIndices, depth + 1);

#ifdef DEBUG_BVH
		totalInternalNodes++;
		if (depth < 5) // Only print deeper nodes for major splits
		{
			std::cout << "[BVH] Internal Node | Depth: " << depth
				<< " | Split Axis: " << axis
				<< " | Left: " << leftIndices.size()
				<< " | Right: " << rightIndices.size()
				<< " | Total Internal Nodes: " << totalInternalNodes << std::endl;
		}

		if (depth == 0) { // Print final statistics after BVH root finishes
			std::cout << "[BVH Stats] Total Internal Nodes: " << totalInternalNodes
				<< " | Total Leaf Nodes: " << totalLeafNodes << std::endl;

			// Validate total triangle count
			int totalTriangles = 0;
			countTriangles(this, totalTriangles);
			std::cout << "[BVH] Total Triangles in Leaves: " << totalTriangles
				<< " (Expected: " << triangles.size() << ")" << std::endl;
		}
#endif
	}

	// Helper function to count triangles in leaf nodes
	void countTriangles(BVHNode* node, int& total)
	{
		if (!node) return;
		if (!node->left && !node->right)
			total += node->triangleIndices.size();
		else
		{
			countTriangles(node->left, total);
			countTriangles(node->right, total);
		}
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		if (!bounds.rayAABB(ray))
			return;

		if (!left && !right) {
			for (int idx : triangleIndices) {
				float t, u, v;
				if (triangles[idx].rayIntersect(ray, t, u, v) && t < intersection.t) {
					intersection.t = t;
					intersection.ID = idx;
					intersection.alpha = u;
					intersection.beta = v;
					intersection.gamma = 1.0f - (u + v);
				}
			}
		}
		else {
			if (left) left->traverse(ray, triangles, intersection);
			if (right) right->traverse(ray, triangles, intersection);
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		if (!bounds.rayAABB(ray))
			return false;

		if (!left && !right)
		{
			for (int idx : triangleIndices)
			{
				float t, u, v;
				if (triangles[idx].rayIntersect(ray, t, u, v)) 
				{
					if (t < maxT)
					{
						return false;
					}
				}
			}
			return true;
		}

		return (left && left->traverseVisible(ray, triangles, maxT)) &&
			(right && right->traverseVisible(ray, triangles, maxT));
	}
};

#ifdef DEBUG_BVH
int BVHNode::totalLeafNodes = 0;
int BVHNode::totalInternalNodes = 0;
#endif