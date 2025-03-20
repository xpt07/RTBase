#include "pch.h"
#include "Geometry.h"

TEST(PlaneRayIntersect, BasicIntersection) {
    Plane p;
    p.init(Vec3(0, 1, 0), 0);

    Ray r;
    r.init(Vec3(0, 10, 0), Vec3(0, -1, 0));

    float t = 0.0f;
    bool intersects = p.rayIntersect(r, t);

    EXPECT_TRUE(intersects);

    // Since the ray starts at y = 10 and hits the plane at y = 0,
    // the intersection should occur at t = 10.
    EXPECT_FLOAT_EQ(t, 10.0f);
}

TEST(TriangleRayIntersect, BasicIntersection) {
    // Define a simple triangle in the XY plane
    Vertex v0 = { Vec3(0, 0, 0) };
    Vertex v1 = { Vec3(1, 0, 0) };
    Vertex v2 = { Vec3(0, 1, 0) };

    Triangle tri;
    tri.init(v0, v1, v2, 0);

    // Define a ray that intersects the triangle
    Ray r;
    r.init(Vec3(0.25f, 0.25f, 1.0f), Vec3(0, 0, -1));

    float t, u, v;
    bool intersects = tri.rayIntersect(r, t, u, v);

    EXPECT_TRUE(intersects);
    EXPECT_GT(t, 0.0f);
    EXPECT_GE(u, 0.0f);
    EXPECT_LE(u, 1.0f);
    EXPECT_GE(v, 0.0f);
    EXPECT_LE(v, 1.0f);
    EXPECT_LE(u + v, 1.0f); // Ensure (u,v) are inside the triangle
}

TEST(TriangleRayIntersect, NoIntersection) {
    // Define a triangle
    Vertex v0 = { Vec3(0, 0, 0) };
    Vertex v1 = { Vec3(1, 0, 0) };
    Vertex v2 = { Vec3(0, 1, 0) };

    Triangle tri;
    tri.init(v0, v1, v2, 0);

    // Define a ray that misses the triangle
    Ray r;
    r.init(Vec3(2, 2, 1), Vec3(0, 0, -1));

    float t, u, v;
    bool intersects = tri.rayIntersect(r, t, u, v);

    EXPECT_FALSE(intersects);
}

TEST(TriangleRayIntersect, ParallelRay) {
    // Define a triangle
    Vertex v0 = { Vec3(0, 0, 0) };
    Vertex v1 = { Vec3(1, 0, 0) };
    Vertex v2 = { Vec3(0, 1, 0) };

    Triangle tri;
    tri.init(v0, v1, v2, 0);

    // Define a ray parallel to the triangle's plane
    Ray r;
    r.init(Vec3(0.5f, 0.5f, 1.0f), Vec3(1, 1, 0));

    float t, u, v;
    bool intersects = tri.rayIntersect(r, t, u, v);

    EXPECT_FALSE(intersects);
}

TEST(TriangleRayIntersect, BehindTriangle) {
    // Define a triangle
    Vertex v0 = { Vec3(0, 0, 0) };
    Vertex v1 = { Vec3(1, 0, 0) };
    Vertex v2 = { Vec3(0, 1, 0) };

    Triangle tri;
    tri.init(v0, v1, v2, 0);

    // Define a ray that points away from the triangle
    Ray r;
    r.init(Vec3(0.25f, 0.25f, -1.0f), Vec3(0, 0, -1));

    float t, u, v;
    bool intersects = tri.rayIntersect(r, t, u, v);

    EXPECT_FALSE(intersects);
}

// AABB Ray Intersection Tests
TEST(AABBRayIntersect, BasicIntersection) {
    AABB box;
    box.min = Vec3(-1, -1, -1);
    box.max = Vec3(1, 1, 1);

    Ray r;
    r.init(Vec3(0, 0, 2), Vec3(0, 0, -1));

    float t;
    bool intersects = box.rayAABB(r, t);

    EXPECT_TRUE(intersects); // Ray should intersect the box along -Z axis
    EXPECT_GT(t, 0.0f); // Intersection t should be positive
}

TEST(AABBRayIntersect, Miss)
{
    AABB box;
    box.min = Vec3(-1, -1, -1);
    box.max = Vec3(1, 1, 1);

    // Ray is off to the side, parallel to Z, never enters the box
    Ray r;
    r.init(Vec3(2, 2, 0), Vec3(0, 0, -1));

    float t = -1.0f;
    bool intersects = box.rayAABB(r, t);

    EXPECT_FALSE(intersects); // Ray should miss the box entirely
}

TEST(AABBRayIntersect, OriginInsideBox)
{
    AABB box;
    box.min = Vec3(-1, -1, -1);
    box.max = Vec3(1, 1, 1);

    // Ray origin is inside the box
    Ray r;
    r.init(Vec3(0, 0, 0), Vec3(0, 0, 1));

    float t = -1.0f;
    bool intersects = box.rayAABB(r, t);

    EXPECT_TRUE(intersects); // Ray origin is inside the box, so it should intersect immediately
    EXPECT_GE(t, 0.0f);
}

TEST(AABBRayIntersect, ParallelToFace)
{
    AABB box;
    box.min = Vec3(-1, -1, -1);
    box.max = Vec3(1, 1, 1);

    // Ray is parallel to the X-axis and outside the box on X, but goes through the YZ-plane
    Ray r;
    r.init(Vec3(-1, 2, 0), Vec3(1, 0, 0));

    float t;
    bool intersects = box.rayAABB(r, t);

    EXPECT_FALSE(intersects) << "Ray is parallel to X and offset in Y, so it should miss the box.";
}

//------------------------------------------
// AABB Extend Tests
//------------------------------------------
TEST(AABBExtend, SinglePoint)
{
    AABB box;
    box.reset();
    EXPECT_GT(box.min.x, box.max.x) << "After reset, min should be +FLT_MAX, max should be -FLT_MAX.";

    Vec3 point(2, 3, 4);
    box.extend(point);

    EXPECT_FLOAT_EQ(box.min.x, point.x);
    EXPECT_FLOAT_EQ(box.min.y, point.y);
    EXPECT_FLOAT_EQ(box.min.z, point.z);

    EXPECT_FLOAT_EQ(box.max.x, point.x);
    EXPECT_FLOAT_EQ(box.max.y, point.y);
    EXPECT_FLOAT_EQ(box.max.z, point.z);

}

TEST(AABBExtend, MultiplePoints)
{
    AABB box;
    box.reset();
    box.extend(Vec3(2, 3, 4));
    box.extend(Vec3(-1, 10, 0));
    box.extend(Vec3(1, 0, 5));

    // After extending those 3 points, the min should be (-1, 0, 0), max should be (2, 10, 5).
    EXPECT_FLOAT_EQ(box.min.x, -1.0f);
    EXPECT_FLOAT_EQ(box.min.y, 0.0f);
    EXPECT_FLOAT_EQ(box.min.z, 0.0f);

    EXPECT_FLOAT_EQ(box.max.x, 2.0f);
    EXPECT_FLOAT_EQ(box.max.y, 10.0f);
    EXPECT_FLOAT_EQ(box.max.z, 5.0f);
}

TEST(AABBExtend, ExtendWithTriangle)
{
    AABB box;
    box.reset();

    Vertex v0 = { Vec3(1, 2, 3), Vec3(0, 0, 1), 0.0f, 0.0f };
    Vertex v1 = { Vec3(-1, 5, 0), Vec3(0, 1, 0), 0.5f, 0.5f };
    Vertex v2 = { Vec3(3, -2, 4), Vec3(1, 0, 0), 1.0f, 1.0f };

    Triangle tri;
    tri.init(v0, v1, v2, 0);

    box.extend(tri);

    EXPECT_FLOAT_EQ(box.min.x, -1.0f);
    EXPECT_FLOAT_EQ(box.min.y, -2.0f);
    EXPECT_FLOAT_EQ(box.min.z, 0.0f);

    EXPECT_FLOAT_EQ(box.max.x, 3.0f);
    EXPECT_FLOAT_EQ(box.max.y, 5.0f);
    EXPECT_FLOAT_EQ(box.max.z, 4.0f);
}


TEST(AABBExtend, MergeBoxes)
{
    AABB boxA;
    boxA.min = Vec3(0, 0, 0);
    boxA.max = Vec3(1, 1, 1);

    AABB boxB;
    boxB.min = Vec3(-2, 0, 0);
    boxB.max = Vec3(0, 5, 5);

    boxA.extend(boxB);

    EXPECT_FLOAT_EQ(boxA.min.x, -2.0f);
    EXPECT_FLOAT_EQ(boxA.min.y, 0.0f);
    EXPECT_FLOAT_EQ(boxA.min.z, 0.0f);

    EXPECT_FLOAT_EQ(boxA.max.x, 1.0f);
    EXPECT_FLOAT_EQ(boxA.max.y, 5.0f);
    EXPECT_FLOAT_EQ(boxA.max.z, 5.0f);
}

//------------------------------------------
// AABB Area Tests
//------------------------------------------
TEST(AABBMethods, Area)
{
    AABB box;
    box.min = Vec3(-1, -2, -3);
    box.max = Vec3(1, 2, 3);

    // The box dimensions: (2, 4, 6).
    // Surface area = 2*(2*4 + 4*6 + 2*6) = 2*(8 + 24 + 12) = 2*44 = 88.
    float expectedArea = 88.0f;
    float area = box.area();

    EXPECT_FLOAT_EQ(area, expectedArea);
}