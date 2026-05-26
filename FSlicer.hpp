#pragma once

#include "FProjection.hpp"

#include "al/graphics/al_Graphics.hpp"
#include "al/graphics/al_Mesh.hpp"
#include "al/types/al_Color.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

using namespace al;

// ----------------------------------------------------------------------------
// Hyperplane slicing (4D -> 3D) at viewer-local w = wPlane.
//
// This is a direct translation of the Processing slicer idea, but implemented
// in a more structured way by slicing *quads* (surface patches) into *segments*.
//
// Given a 4D surface described as quads (each quad is 4 vertices around its
// boundary), intersect that surface with the hyperplane:
//   local_w == wPlane
//
// where "local" is viewer-local 4D coordinates:
//   local = viewerRot^{-1} * (world - viewer.pos)
//
// Each quad intersection yields:
// - 0 points: no intersection
// - 2 points: one segment
// - 4 points: two segments (rare, but possible when the plane cuts twice)
// ----------------------------------------------------------------------------

struct SliceSettings
{
    float wPlane{0.0f};
    float sliceScale{25.0f}; // Processing used x25 for visibility
    float denomEps{1e-5f};

    bool drawPoints{true};
    float pointSize{4.0f};

    al::Color color{al::Color(0.0f, 1.0f, 0.8f, 1.0f)}; // cyan-ish
};

struct SliceResult
{
    al::Mesh segments;
    al::Mesh points;
};

struct HyperSliceSettings
{
    float wPlane{0.0f};
    float sliceScale{25.0f};
    float denomEps{1e-5f};

    // Rendering
    al::Color fillColor{al::Color(0.0f, 0.8f, 1.0f, 0.18f)};
    al::Color edgeColor{al::Color(1.0f, 1.0f, 1.0f, 0.20f)};
    al::Color pointColor{al::Color(1.0f, 1.0f, 1.0f, 0.55f)};
    bool drawEdges{true};
    bool drawPoints{true};
    float pointSize{3.5f};

    // Quantization for internal-face cancellation
    float quant{1e-3f};
};

struct HyperSliceResult
{
    al::Mesh triangles;
    al::Mesh edges;
    al::Mesh points;
};

inline bool edgeIntersectsPlane(float w0, float w1, float wPlane)
{
    return (w0 <= wPlane && w1 >= wPlane) || (w0 >= wPlane && w1 <= wPlane);
}

inline al::Vec3f slicePointTo3D(const Vec4f &pLocal, float sliceScale)
{
    return al::Vec3f(pLocal.x * sliceScale, pLocal.y * sliceScale, pLocal.z * sliceScale);
}

inline SliceResult sliceQuadsViewerLocal(
    const Object4D &viewer,
    const std::vector<Vec4f> &vertsWorld,
    const std::vector<std::array<int, 4>> &quads,
    const SliceSettings &settings)
{
    SliceResult out;
    out.segments.primitive(al::Mesh::LINES);
    out.points.primitive(al::Mesh::POINTS);

    if (vertsWorld.empty() || quads.empty())
    {
        return out;
    }

    // Precompute viewer-local vertices so slicing is consistent with the viewer.
    std::vector<Vec4f> vertsLocal;
    vertsLocal.resize(vertsWorld.size());
    for (size_t i = 0; i < vertsWorld.size(); ++i)
    {
        vertsLocal[i] = toViewerLocal(viewer, vertsWorld[i]);
    }

    // Edges on quad perimeter in winding order (0-1-2-3-0).
    static const int edgeA[4] = {0, 1, 2, 3};
    static const int edgeB[4] = {1, 2, 3, 0};

    for (const auto &q : quads)
    {
        Vec4f p[4] = {
            vertsLocal[q[0]],
            vertsLocal[q[1]],
            vertsLocal[q[2]],
            vertsLocal[q[3]],
        };

        // Collect intersection points along quad boundary.
        al::Vec3f isect[4];
        int isectCount = 0;

        for (int e = 0; e < 4; ++e)
        {
            const Vec4f &a = p[edgeA[e]];
            const Vec4f &b = p[edgeB[e]];

            if (!edgeIntersectsPlane(a.w, b.w, settings.wPlane))
            {
                continue;
            }

            const float denom = (b.w - a.w);
            if (std::fabs(denom) <= settings.denomEps)
            {
                continue;
            }

            const float t = (settings.wPlane - a.w) / denom;
            if (t < 0.0f || t > 1.0f)
            {
                continue;
            }

            const Vec4f p4 = a + (b - a) * t;
            isect[isectCount++] = slicePointTo3D(p4, settings.sliceScale);
            if (isectCount == 4)
            {
                break;
            }
        }

        if (isectCount == 2)
        {
            out.segments.color(settings.color);
            out.segments.vertex(isect[0]);
            out.segments.color(settings.color);
            out.segments.vertex(isect[1]);

            if (settings.drawPoints)
            {
                out.points.color(settings.color);
                out.points.vertex(isect[0]);
                out.points.color(settings.color);
                out.points.vertex(isect[1]);
            }
        }
        else if (isectCount == 4)
        {
            // Connect in order along perimeter: (0-1) and (2-3).
            out.segments.color(settings.color);
            out.segments.vertex(isect[0]);
            out.segments.color(settings.color);
            out.segments.vertex(isect[1]);

            out.segments.color(settings.color);
            out.segments.vertex(isect[2]);
            out.segments.color(settings.color);
            out.segments.vertex(isect[3]);

            if (settings.drawPoints)
            {
                for (int i = 0; i < 4; ++i)
                {
                    out.points.color(settings.color);
                    out.points.vertex(isect[i]);
                }
            }
        }
    }

    return out;
}

inline void drawSliceResult(al::Graphics &g, const SliceResult &r, const SliceSettings &settings)
{
    g.depthTesting(true);
    g.meshColor();
    g.draw(r.segments);

    if (settings.drawPoints)
    {
        g.pointSize(settings.pointSize);
        g.meshColor();
        g.draw(r.points);
    }
}

// ----------------------------------------------------------------------------
// Exact hypervolume slicing: 4-simplices -> boundary triangle mesh
// ----------------------------------------------------------------------------

struct QuantKey3
{
    int32_t x, y, z;
    bool operator==(const QuantKey3 &o) const { return x == o.x && y == o.y && z == o.z; }
};

struct QuantKeyTri
{
    QuantKey3 a, b, c;
    bool operator==(const QuantKeyTri &o) const
    {
        return a == o.a && b == o.b && c == o.c;
    }
};

struct QuantKeyTriHash
{
    size_t operator()(const QuantKeyTri &k) const
    {
        auto h = [](int32_t v) -> size_t
        { return static_cast<uint32_t>(v) * 2654435761u; };
        size_t r = h(k.a.x) ^ (h(k.a.y) << 1) ^ (h(k.a.z) << 2);
        r ^= (h(k.b.x) << 3) ^ (h(k.b.y) << 4) ^ (h(k.b.z) << 5);
        r ^= (h(k.c.x) << 6) ^ (h(k.c.y) << 7) ^ (h(k.c.z) << 8);
        return r;
    }
};

inline QuantKey3 quantize3(const al::Vec3f &p, float q)
{
    const float inv = 1.0f / q;
    return QuantKey3{
        static_cast<int32_t>(std::lround(p.x * inv)),
        static_cast<int32_t>(std::lround(p.y * inv)),
        static_cast<int32_t>(std::lround(p.z * inv))};
}

inline QuantKeyTri makeTriKey(const al::Vec3f &p0, const al::Vec3f &p1, const al::Vec3f &p2, float q)
{
    QuantKey3 k0 = quantize3(p0, q);
    QuantKey3 k1 = quantize3(p1, q);
    QuantKey3 k2 = quantize3(p2, q);

    // sort lexicographically
    auto less = [](const QuantKey3 &a, const QuantKey3 &b)
    {
        if (a.x != b.x)
            return a.x < b.x;
        if (a.y != b.y)
            return a.y < b.y;
        return a.z < b.z;
    };

    QuantKey3 a = k0, b = k1, c = k2;
    if (less(b, a))
        std::swap(a, b);
    if (less(c, b))
        std::swap(b, c);
    if (less(b, a))
        std::swap(a, b);

    return QuantKeyTri{a, b, c};
}

inline void emitEdgesFromTriangles(al::Mesh &edges, const al::Mesh &tris, const al::Color &c)
{
    edges.primitive(al::Mesh::LINES);
    edges.reset();

    const auto &v = tris.vertices();
    for (size_t i = 0; i + 2 < v.size(); i += 3)
    {
        edges.color(c);
        edges.vertex(v[i + 0]);
        edges.color(c);
        edges.vertex(v[i + 1]);

        edges.color(c);
        edges.vertex(v[i + 1]);
        edges.color(c);
        edges.vertex(v[i + 2]);

        edges.color(c);
        edges.vertex(v[i + 2]);
        edges.color(c);
        edges.vertex(v[i + 0]);
    }
}

inline HyperSliceResult slice4SimplicesViewerLocal(
    const Object4D &viewer,
    const std::vector<Vec4f> &vertsWorld,
    const std::vector<std::array<int, 5>> &simplices,
    const HyperSliceSettings &settings)
{
    HyperSliceResult out;
    out.triangles.primitive(al::Mesh::TRIANGLES);
    out.edges.primitive(al::Mesh::LINES);
    out.points.primitive(al::Mesh::POINTS);

    if (vertsWorld.empty() || simplices.empty())
    {
        return out;
    }

    // Viewer-local vertices
    std::vector<Vec4f> vertsLocal;
    vertsLocal.resize(vertsWorld.size());
    for (size_t i = 0; i < vertsWorld.size(); ++i)
    {
        vertsLocal[i] = toViewerLocal(viewer, vertsWorld[i]);
    }

    // Edge list of a 4-simplex (5 vertices -> 10 edges)
    static const int EA[10] = {0, 0, 0, 0, 1, 1, 1, 2, 2, 3};
    static const int EB[10] = {1, 2, 3, 4, 2, 3, 4, 3, 4, 4};

    struct Tri
    {
        al::Vec3f a, b, c;
    };

    std::vector<Tri> tris;
    tris.reserve(simplices.size() * 6);

    // For each simplex: collect intersection points on edges; then triangulate the
    // convex hull boundary of the intersection polyhedron.
    for (const auto &s : simplices)
    {
        Vec4f p4[5] = {
            vertsLocal[s[0]],
            vertsLocal[s[1]],
            vertsLocal[s[2]],
            vertsLocal[s[3]],
            vertsLocal[s[4]]};

        al::Vec3f ip[10];
        int ipCount = 0;

        for (int e = 0; e < 10; ++e)
        {
            const Vec4f &a = p4[EA[e]];
            const Vec4f &b = p4[EB[e]];
            if (!edgeIntersectsPlane(a.w, b.w, settings.wPlane))
            {
                continue;
            }
            const float denom = b.w - a.w;
            if (std::fabs(denom) <= settings.denomEps)
            {
                continue;
            }
            const float t = (settings.wPlane - a.w) / denom;
            if (t < 0.0f || t > 1.0f)
            {
                continue;
            }
            const Vec4f q4 = a + (b - a) * t;
            ip[ipCount++] = slicePointTo3D(q4, settings.sliceScale);
            if (ipCount == 10)
                break;
        }

        // Deduplicate points by quantization.
        std::vector<al::Vec3f> pts;
        pts.reserve(static_cast<size_t>(ipCount));
        std::unordered_map<uint64_t, int> seen;
        seen.reserve(static_cast<size_t>(ipCount) * 2);
        for (int i = 0; i < ipCount; ++i)
        {
            const QuantKey3 qk = quantize3(ip[i], settings.quant);
            const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(qk.x)) << 42) ^ (static_cast<uint64_t>(static_cast<uint32_t>(qk.y)) << 21) ^ static_cast<uint64_t>(static_cast<uint32_t>(qk.z));
            if (seen.find(key) == seen.end())
            {
                seen[key] = static_cast<int>(pts.size());
                pts.push_back(ip[i]);
            }
        }

        if (pts.size() < 4)
        {
            continue; // not a 3D polyhedron
        }

        // Convex hull brute-force for <= 5 points:
        // For every triple (i,j,k), determine if it forms a hull face by checking
        // all other points are on one side of the plane.
        const float eps = 1e-5f;
        const int n = static_cast<int>(pts.size());
        for (int i = 0; i < n; ++i)
        {
            for (int j = i + 1; j < n; ++j)
            {
                for (int k = j + 1; k < n; ++k)
                {
                    const al::Vec3f a = pts[i];
                    const al::Vec3f b = pts[j];
                    const al::Vec3f c = pts[k];

                    const al::Vec3f nrm = al::cross(b - a, c - a);
                    const float nl = nrm.mag();
                    if (nl < eps)
                    {
                        continue;
                    }

                    int posSide = 0;
                    int negSide = 0;
                    for (int m = 0; m < n; ++m)
                    {
                        if (m == i || m == j || m == k)
                            continue;
                        const float d = al::dot(nrm, pts[m] - a);
                        if (d > eps)
                            posSide++;
                        else if (d < -eps)
                            negSide++;
                    }

                    if (posSide && negSide)
                    {
                        continue; // not a hull face
                    }

                    // Pick a consistent orientation (doesn't matter for our cancellation key)
                    tris.push_back(Tri{a, b, c});
                }
            }
        }
    }

    // Cancel internal faces by counting identical triangles (quantized, sorted vertices).
    std::unordered_map<QuantKeyTri, int, QuantKeyTriHash> triCounts;
    triCounts.reserve(tris.size() * 2);
    for (const auto &t : tris)
    {
        const auto key = makeTriKey(t.a, t.b, t.c, settings.quant);
        triCounts[key] += 1;
    }

    for (const auto &t : tris)
    {
        const auto key = makeTriKey(t.a, t.b, t.c, settings.quant);
        if (triCounts[key] != 1)
        {
            continue;
        }
        out.triangles.color(settings.fillColor);
        out.triangles.vertex(t.a);
        out.triangles.color(settings.fillColor);
        out.triangles.vertex(t.b);
        out.triangles.color(settings.fillColor);
        out.triangles.vertex(t.c);
    }

    if (settings.drawEdges)
    {
        emitEdgesFromTriangles(out.edges, out.triangles, settings.edgeColor);
    }

    if (settings.drawPoints)
    {
        out.points.reset();
        out.points.primitive(al::Mesh::POINTS);
        for (const auto &v : out.triangles.vertices())
        {
            out.points.color(settings.pointColor);
            out.points.vertex(v);
        }
    }

    return out;
}

inline void drawHyperSlice(al::Graphics &g, const HyperSliceResult &r, const HyperSliceSettings &settings)
{
    g.depthTesting(true);
    g.blending(true);
    g.blendTrans();
    g.meshColor();
    g.draw(r.triangles);
    g.blending(false);

    if (settings.drawEdges)
    {
        g.meshColor();
        g.draw(r.edges);
    }

    if (settings.drawPoints)
    {
        g.pointSize(settings.pointSize);
        g.meshColor();
        g.draw(r.points);
    }
}
