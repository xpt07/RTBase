#pragma once

#include "Core.h"
#include "Sampling.h"
#include <stack>

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

#define MollEPSILON 1e-7f
#define EPSILON 0.001f

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
		Vec3 pVec = Cross(r.dir, e2);
		float determinant = Dot(pVec, e1);

		if (std::fabs(determinant) < MollEPSILON)
			return false;

		float invDet = 1.0f / determinant;
		Vec3 tVec = r.o - vertices[2].p;

		u = Dot(tVec, pVec) * invDet;
		if (u < -MollEPSILON || u > 1.0f + MollEPSILON)
			return false;

		Vec3 qVec = Cross(tVec, e1);
		v = Dot(r.dir, qVec) * invDet;
		if (v < -MollEPSILON || (u + v) > 1.0f + MollEPSILON)
			return false;

		t = Dot(e2, qVec) * invDet;
		if (t < MollEPSILON)
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
	bool rayAABB(const Ray& r, float& t) const
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

	bool rayAABB(const Ray& r) const
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
    bool hit;
};

// Constants controlling BVH behaviour.  Adjust them to tune leaf thresholds and bin counts.
static const int MAX_NODE_TRIANGLES = 6;
static const float TRAVERSE_COST = 1.0f;
static const float TRIANGLE_COST = 2.0f;
static const int BUILD_BINS = 16; // number of SAH bins per axis
static const int MAX_DEPTH = 32;

class BVHNode {
public:
    AABB bounds;                    // bounding box of this node
    BVHNode* left = nullptr;       // child pointer
    BVHNode* right = nullptr;       // child pointer
    std::vector<int> triangleIndices; // indices of primitives for leaf nodes

    ~BVHNode() {
        delete left;
        delete right;
    }

    /**
     * Recursively build the BVH using a simple Surface Area Heuristic (SAH)
     * with fixed bins.  The input "indices" is a list of triangle indices into
     * the global triangles array.  The node will either become a leaf (if
     * small enough or at max depth) or split into two children based on a
     * SAH‑chosen plane.  Depth limits prevent stack overflow on degenerate
     * scenes.
     */
    void build(const std::vector<Triangle>& triangles,
        std::vector<int>& indices,
        int depth = 0)
    {
        // Compute bounding box for this node
        bounds.reset();
        for (int i : indices) {
            bounds.extend(triangles[i]);
        }

        // Base case: if few primitives or max depth reached, make a leaf
        if ((int)indices.size() <= MAX_NODE_TRIANGLES || depth >= MAX_DEPTH) {
            triangleIndices = std::move(indices);
            return;
        }

        // Compute the surface area of the current node for SAH
        float parentArea = bounds.area();
        // If the parent area is zero (degenerate), fall back to leaf
        if (parentArea <= 0.0f) {
            triangleIndices = std::move(indices);
            return;
        }

        // Arrays to hold bin data for each axis
        struct BinInfo {
            AABB box;
            int count = 0;
        };
        std::vector<BinInfo> binsX(BUILD_BINS), binsY(BUILD_BINS), binsZ(BUILD_BINS);

        // Precompute extents along each axis
        Vec3 minP = bounds.min;
        Vec3 maxP = bounds.max;
        float dx = maxP.x - minP.x;
        float dy = maxP.y - minP.y;
        float dz = maxP.z - minP.z;

        // Avoid division by zero; if extent is zero, we cannot split along that axis
        bool canSplitX = dx > 1e-6f;
        bool canSplitY = dy > 1e-6f;
        bool canSplitZ = dz > 1e-6f;

        // If no axis can split, create leaf
        if (!canSplitX && !canSplitY && !canSplitZ) {
            triangleIndices = std::move(indices);
            return;
        }

        // Assign triangles to bins for each axis
        for (int idx : indices) {
            Vec3 c = triangles[idx].centre();
            if (canSplitX) {
                int bin = std::min((int)((c.x - minP.x) / dx * (float)BUILD_BINS), BUILD_BINS - 1);
                binsX[bin].count++;
                binsX[bin].box.extend(triangles[idx]);
            }
            if (canSplitY) {
                int bin = std::min((int)((c.y - minP.y) / dy * (float)BUILD_BINS), BUILD_BINS - 1);
                binsY[bin].count++;
                binsY[bin].box.extend(triangles[idx]);
            }
            if (canSplitZ) {
                int bin = std::min((int)((c.z - minP.z) / dz * (float)BUILD_BINS), BUILD_BINS - 1);
                binsZ[bin].count++;
                binsZ[bin].box.extend(triangles[idx]);
            }
        }

        // Helper lambda to evaluate SAH for an axis
        auto evaluateSAH = [&](std::vector<BinInfo>& bins, bool enabled) {
            if (!enabled) return std::make_pair(std::numeric_limits<float>::infinity(), -1);
            // Prefix sums for left side
            std::vector<float> leftArea(BUILD_BINS);
            std::vector<int> leftCount(BUILD_BINS);
            AABB curBox;
            curBox.reset();
            int count = 0;
            for (int i = 0; i < BUILD_BINS; ++i) {
                if (bins[i].count > 0) {
                    curBox.extend(bins[i].box);
                    count += bins[i].count;
                }
                leftArea[i] = count > 0 ? curBox.area() : 0.0f;
                leftCount[i] = count;
            }
            // Suffix sums for right side
            std::vector<float> rightArea(BUILD_BINS);
            std::vector<int> rightCount(BUILD_BINS);
            curBox.reset();
            count = 0;
            for (int i = BUILD_BINS - 1; i >= 0; --i) {
                if (bins[i].count > 0) {
                    curBox.extend(bins[i].box);
                    count += bins[i].count;
                }
                rightArea[i] = count > 0 ? curBox.area() : 0.0f;
                rightCount[i] = count;
            }
            // Evaluate cost for each possible split after bin i
            float bestCost = std::numeric_limits<float>::infinity();
            int bestSplit = -1;
            for (int i = 0; i < BUILD_BINS - 1; ++i) {
                int nLeft = leftCount[i];
                int nRight = rightCount[i + 1];
                if (nLeft == 0 || nRight == 0) continue;
                float cost = TRAVERSE_COST +
                    (leftArea[i] / parentArea) * nLeft * TRIANGLE_COST +
                    (rightArea[i + 1] / parentArea) * nRight * TRIANGLE_COST;
                if (cost < bestCost) {
                    bestCost = cost;
                    bestSplit = i;
                }
            }
            return std::make_pair(bestCost, bestSplit);
            };

        // Evaluate SAH for each axis and choose the best
        auto sahX = evaluateSAH(binsX, canSplitX);
        auto sahY = evaluateSAH(binsY, canSplitY);
        auto sahZ = evaluateSAH(binsZ, canSplitZ);

        float bestCost = sahX.first;
        int bestAxis = 0;
        int bestSplit = sahX.second;
        if (sahY.first < bestCost) {
            bestCost = sahY.first;
            bestAxis = 1;
            bestSplit = sahY.second;
        }
        if (sahZ.first < bestCost) {
            bestCost = sahZ.first;
            bestAxis = 2;
            bestSplit = sahZ.second;
        }

        // Compute the cost of not splitting (leaf cost)
        float leafCost = TRIANGLE_COST * indices.size();
        // If splitting does not improve cost, make a leaf
        if (bestSplit < 0 || bestCost >= leafCost) {
            triangleIndices = std::move(indices);
            return;
        }

        // Determine split position along the chosen axis
        float axisMin;
        float axisExtent;
        if (bestAxis == 0) {
            axisMin = minP.x;
            axisExtent = dx;
        }
        else if (bestAxis == 1) {
            axisMin = minP.y;
            axisExtent = dy;
        }
        else {
            axisMin = minP.z;
            axisExtent = dz;
        }
        float binSize = axisExtent / (float)BUILD_BINS;
        float splitPos = axisMin + binSize * (bestSplit + 1);

        // Partition indices into left and right based on the split position
        std::vector<int> leftIndices;
        std::vector<int> rightIndices;
        leftIndices.reserve(indices.size());
        rightIndices.reserve(indices.size());
        for (int idx : indices) {
            float c;
            if (bestAxis == 0)      c = triangles[idx].centre().x;
            else if (bestAxis == 1) c = triangles[idx].centre().y;
            else                    c = triangles[idx].centre().z;
            if (c < splitPos) {
                leftIndices.push_back(idx);
            }
            else {
                rightIndices.push_back(idx);
            }
        }
        // If partitioning produced an empty side, fall back to median split
        if (leftIndices.empty() || rightIndices.empty()) {
            // Median split along the longest axis
            int axis = 0;
            Vec3 ext = { dx, dy, dz };
            if (ext.x >= ext.y && ext.x >= ext.z) {
                axis = 0;
            }
            else if (ext.y >= ext.z) {
                axis = 1;
            }
            else {
                axis = 2;
            }
            // Sort indices by centroid along the chosen axis
            std::sort(indices.begin(), indices.end(), [&](int a, int b) {
                return triangles[a].centre()[axis] < triangles[b].centre()[axis];
                });
            size_t mid = indices.size() / 2;
            leftIndices.assign(indices.begin(), indices.begin() + mid);
            rightIndices.assign(indices.begin() + mid, indices.end());
        }

        // Create child nodes and recursively build them
        left = new BVHNode();
        right = new BVHNode();
        left->build(triangles, leftIndices, depth + 1);
        right->build(triangles, rightIndices, depth + 1);
    }

    // Traversal (hit testing)
    bool traverse(const std::vector<Triangle>& triangles, const Ray& ray, float& tHit, int& triIndex) const {
        float tBox;
        if (!bounds.rayAABB(ray, tBox)) return false;

        bool hit = false;
        float closest = tHit;

        if (!left && !right) {
            for (int idx : triangleIndices) {
                float t, u, v;
                if (triangles[idx].rayIntersect(ray, t, u, v) && t < closest) {
                    closest = t;
                    triIndex = idx;
                    hit = true;
                }
            }
            tHit = closest;
            return hit;
        }

        if (left)  hit |= left->traverse(triangles, ray, tHit, triIndex);
        if (right) hit |= right->traverse(triangles, ray, tHit, triIndex);

        return hit;
    }

    bool traverseVisible(const std::vector<Triangle>& triangles, const Ray& ray, float maxDist) const {
        float tBox;
        if (!bounds.rayAABB(ray, tBox) || tBox > maxDist) return false;

        if (!left && !right) {
            for (int idx : triangleIndices) {
                float t, u, v;
                if (triangles[idx].rayIntersect(ray, t, u, v) && t < maxDist) {
                    return true;
                }
            }
            return false;
        }

        if (left && left->traverseVisible(triangles, ray, maxDist)) return true;
        if (right && right->traverseVisible(triangles, ray, maxDist)) return true;
        return false;
    }

    IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles) const {
        IntersectionData hit;
        hit.t = FLT_MAX;
        hit.ID = -1;
        hit.hit = false;

        float t = FLT_MAX;
        int triIndex = -1;
        if (traverse(triangles, ray, t, triIndex)) {
            hit.t = t;
            hit.ID = triIndex;
            hit.hit = true;

            const Triangle& tri = triangles[triIndex];
            float u, v;
            tri.rayIntersect(ray, t, u, v);
            hit.alpha = 1.0f - u - v;
            hit.beta = u;
            hit.gamma = v;
        }
        return hit;
    }

    bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, float maxT) const {
        return traverseVisible(triangles, ray, maxT);
    }
};