#pragma once

#include "Manifolds4D.hpp"
#include "al/io/al_Imgui.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

using namespace al;

// ----------------------------------------------------------------------------
// True 4D Nautilus (hypervolume) built as a swept 3D-ball through 4D.
//
// - Centerline: the same 4D logarithmic spiral as the Processing nautilus.
// - Cross-section: a 3D ball (in the 3D subspace orthogonal to the tangent),
//   approximated by a low-resolution tetrahedral mesh.
// - Sweep: each 3D tetra extruded along the segment between consecutive rings,
//   triangulated into 4-simplices (pentachora). This gives explicit 4D cell
//   connectivity (hypervolume).
//
// Slicing a hypervolume by a hyperplane (w = const in viewer-local coords)
// produces a 3D volume; we render its boundary as triangles from exact simplex
// intersections (implemented in `Fslicer.hpp`).
// ----------------------------------------------------------------------------
class NautilusHypervolume4D : public object4D
{
public:
    struct BuildSettings
    {
        int tSteps{120}; // along spiral
        float maxT{90.0f};

        // Spiral constants + multipliers
        float m1{1.0f};
        float m2{1.0f};
        float m3{1.0f};

        // Growth
        float a{0.05f};
        float b{0.06f};
        float tubeGrow{0.06f};
        float tubeScale{0.6f};

        // Ball mesh resolution (cubic grid in [-1,1]^3)
        int gridRes{3}; // 3 => (4^3) grid points; tetra per cube = 5
        float insideEps{1e-4f};

        // Tangent sampling delta for orthonormal frame
        float tangentDelta{0.01f};
    };

    NautilusHypervolume4D()
    {
        rebuild();
    }

    void rebuild()
    {
        buildTemplateBallTetMesh();
        buildHypervolume();
    }

    void drawImGuiControls()
    {
        ImGui::Separator();
        ImGui::Text("4D Nautilus (hypervolume)");

        bool dirty = false;
        dirty |= ImGui::SliderFloat("m1 (GR)", &settings_.m1, 0.1f, 10.0f, "%.3f");
        dirty |= ImGui::SliderFloat("m2 (E)", &settings_.m2, 0.1f, 10.0f, "%.3f");
        dirty |= ImGui::SliderFloat("m3 (PI)", &settings_.m3, 0.1f, 10.0f, "%.3f");

        dirty |= ImGui::SliderInt("tSteps", &settings_.tSteps, 30, 240);
        dirty |= ImGui::SliderInt("ball gridRes", &settings_.gridRes, 2, 6);

        if (dirty)
        {
            rebuild();
        }
    }

    // Export world-space 4D vertices and 4-simplices (each simplex has 5 vertex indices).
    void buildWorldSimplices(std::vector<Vec4f> &vertsWorld, std::vector<std::array<int, 5>> &simplices) const
    {
        vertsWorld.clear();
        simplices.clear();

        vertsWorld.reserve(verticesLocal_.size());
        for (const auto &vLoc : verticesLocal_)
        {
            vertsWorld.push_back(pos + rotationState.apply(vLoc));
        }

        simplices = simplices_;
    }

    // Projection shadow: draw a wireframe of the outer boundary surface we already
    // have as quads (same as the old surface nautilus).
    void drawProjectedShadow(al::Graphics &g, const object4D &viewer) const
    {
        if (shadowVertsLocal_.empty())
        {
            return;
        }

        al::Mesh mesh;
        mesh.primitive(al::Mesh::LINES);

        for (const auto &e : shadowEdges_)
        {
            const Vec4f aWorld = pos + rotationState.apply(shadowVertsLocal_[e.first]);
            const Vec4f bWorld = pos + rotationState.apply(shadowVertsLocal_[e.second]);

            const Vec4f aLocal = toViewerLocal(viewer, aWorld);
            const Vec4f bLocal = toViewerLocal(viewer, bWorld);

            const al::Vec3f pa = projectLocal4Dto3D(aLocal);
            const al::Vec3f pb = projectLocal4Dto3D(bLocal);

            const al::Color ca = colorFromLocalW(aLocal.w);
            const al::Color cb = colorFromLocalW(bLocal.w);

            mesh.color(ca);
            mesh.vertex(pa);
            mesh.color(cb);
            mesh.vertex(pb);
        }

        g.meshColor();
        g.draw(mesh);
    }

    const BuildSettings &settings() const { return settings_; }

private:
    BuildSettings settings_{};

    // Ball template (unit ball in 3D) expressed as Vec3f points in [-1,1].
    std::vector<al::Vec3f> ballVerts_;
    std::vector<std::array<int, 4>> ballTets_;

    // Hypervolume mesh in local 4D:
    std::vector<Vec4f> verticesLocal_;
    std::vector<std::array<int, 5>> simplices_;

    // Shadow wireframe surface (local 4D)
    std::vector<Vec4f> shadowVertsLocal_;
    std::vector<std::pair<int, int>> shadowEdges_;

    static float norm4(const Vec4f &v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
    }

    static float dot4(const Vec4f &a, const Vec4f &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    // Spiral point (same as Processing).
    Vec4f spiralPoint(float t) const
    {
        const float GR = (1.0f + std::sqrt(5.0f)) / 2.0f;
        const float E = 2.718281828f;
        const float PI = 3.14159265358979323846f;

        const float r = settings_.a * std::exp(settings_.b * t);
        const float theta = (GR * settings_.m1) * t;
        const float phi = (E * settings_.m2) * t;
        const float chi = (PI * settings_.m3) * t;

        return Vec4f(
            r * std::cos(chi),
            r * std::sin(chi) * std::cos(phi),
            r * std::sin(chi) * std::sin(phi) * std::cos(theta),
            r * std::sin(chi) * std::sin(phi) * std::sin(theta));
    }

    float ringT(int ringIndex) const
    {
        if (settings_.tSteps <= 1)
        {
            return 0.0f;
        }
        const float u = static_cast<float>(ringIndex) / static_cast<float>(settings_.tSteps - 1);
        return u * settings_.maxT;
    }

    float tubeRadius(float t) const
    {
        return (settings_.a * std::exp(settings_.tubeGrow * t)) * settings_.tubeScale;
    }

    // Build an orthonormal frame (n1,n2,n3) orthogonal to the tangent.
    void buildOrthonormal3(const Vec4f &tangent, Vec4f &n1, Vec4f &n2, Vec4f &n3) const
    {
        // Start from 3 seed axes; project out tangent and previous normals (Gram-Schmidt).
        Vec4f seeds[3] = {Vec4f(1, 0, 0, 0), Vec4f(0, 1, 0, 0), Vec4f(0, 0, 1, 0)};

        // If tangent is too aligned with x, swap first seed.
        if (std::fabs(tangent.x) > 0.9f)
        {
            seeds[0] = Vec4f(0, 1, 0, 0);
        }

        auto orth = [](const Vec4f &v, const Vec4f &b) -> Vec4f
        {
            return v - b * dot4(v, b);
        };

        n1 = orth(seeds[0], tangent);
        float m = norm4(n1);
        if (m < 1e-6f)
            n1 = Vec4f(0, 0, 1, 0), m = norm4(n1);
        n1 *= (1.0f / m);

        n2 = orth(seeds[1], tangent);
        n2 = orth(n2, n1);
        m = norm4(n2);
        if (m < 1e-6f)
            n2 = Vec4f(0, 0, 0, 1), m = norm4(n2);
        n2 *= (1.0f / m);

        n3 = orth(seeds[2], tangent);
        n3 = orth(n3, n1);
        n3 = orth(n3, n2);
        m = norm4(n3);
        if (m < 1e-6f)
            n3 = Vec4f(0, 0, 0, 1), m = norm4(n3);
        n3 *= (1.0f / m);
    }

    void buildTemplateBallTetMesh()
    {
        const int res = std::max(2, settings_.gridRes);
        const int n = res + 1;

        // Map 3D grid index -> compact vertex index (only keep inside-sphere points).
        std::vector<int> map;
        map.resize(static_cast<size_t>(n) * static_cast<size_t>(n) * static_cast<size_t>(n), -1);

        ballVerts_.clear();
        ballTets_.clear();

        auto idx3 = [n](int ix, int iy, int iz) -> size_t
        {
            return static_cast<size_t>(ix) + static_cast<size_t>(n) * (static_cast<size_t>(iy) + static_cast<size_t>(n) * static_cast<size_t>(iz));
        };

        for (int iz = 0; iz < n; ++iz)
        {
            const float z = -1.0f + 2.0f * static_cast<float>(iz) / static_cast<float>(res);
            for (int iy = 0; iy < n; ++iy)
            {
                const float y = -1.0f + 2.0f * static_cast<float>(iy) / static_cast<float>(res);
                for (int ix = 0; ix < n; ++ix)
                {
                    const float x = -1.0f + 2.0f * static_cast<float>(ix) / static_cast<float>(res);
                    const float r2 = x * x + y * y + z * z;
                    if (r2 <= 1.0f + settings_.insideEps)
                    {
                        const int id = static_cast<int>(ballVerts_.size());
                        map[idx3(ix, iy, iz)] = id;
                        ballVerts_.push_back(al::Vec3f(x, y, z));
                    }
                }
            }
        }

        // Tetrahedralize each cube with a fixed 5-tet pattern.
        // For each cube, if all its 8 corners are present (inside sphere), emit tets.
        for (int iz = 0; iz < res; ++iz)
        {
            for (int iy = 0; iy < res; ++iy)
            {
                for (int ix = 0; ix < res; ++ix)
                {
                    const int c000 = map[idx3(ix, iy, iz)];
                    const int c100 = map[idx3(ix + 1, iy, iz)];
                    const int c010 = map[idx3(ix, iy + 1, iz)];
                    const int c110 = map[idx3(ix + 1, iy + 1, iz)];
                    const int c001 = map[idx3(ix, iy, iz + 1)];
                    const int c101 = map[idx3(ix + 1, iy, iz + 1)];
                    const int c011 = map[idx3(ix, iy + 1, iz + 1)];
                    const int c111 = map[idx3(ix + 1, iy + 1, iz + 1)];

                    if (c000 < 0 || c100 < 0 || c010 < 0 || c110 < 0 || c001 < 0 || c101 < 0 || c011 < 0 || c111 < 0)
                    {
                        continue;
                    }

                    // 5-tet decomposition of a cube (one of many valid patterns).
                    ballTets_.push_back({c000, c100, c010, c001});
                    ballTets_.push_back({c100, c110, c010, c111});
                    ballTets_.push_back({c100, c010, c001, c111});
                    ballTets_.push_back({c010, c001, c011, c111});
                    ballTets_.push_back({c100, c001, c101, c111});
                }
            }
        }
    }

    void buildHypervolume()
    {
        verticesLocal_.clear();
        simplices_.clear();
        shadowVertsLocal_.clear();
        shadowEdges_.clear();

        const int tSteps = std::max(2, settings_.tSteps);
        const int baseV = static_cast<int>(ballVerts_.size());

        verticesLocal_.reserve(static_cast<size_t>(tSteps) * static_cast<size_t>(baseV));

        // Build vertices per ring.
        for (int i = 0; i < tSteps; ++i)
        {
            const float t = ringT(i);
            const Vec4f c = spiralPoint(t);

            // Tangent
            const Vec4f c2 = spiralPoint(t + settings_.tangentDelta);
            Vec4f tan = c2 - c;
            const float tm = norm4(tan);
            if (tm > 1e-6f)
            {
                tan *= (1.0f / tm);
            }

            Vec4f n1, n2, n3;
            buildOrthonormal3(tan, n1, n2, n3);

            const float r = tubeRadius(t);

            for (const auto &p3 : ballVerts_)
            {
                const Vec4f p4 = c + (n1 * (r * p3.x)) + (n2 * (r * p3.y)) + (n3 * (r * p3.z));
                verticesLocal_.push_back(p4);
            }
        }

        // Build 4-simplices by sweeping each 3D tetra along i->i+1 and triangulating tetra×segment.
        const int segs = tSteps - 1;
        simplices_.reserve(static_cast<size_t>(segs) * ballTets_.size() * 4);

        for (int i = 0; i < segs; ++i)
        {
            const int offA = i * baseV;
            const int offB = (i + 1) * baseV;

            for (const auto &tet : ballTets_)
            {
                const int a0 = offA + tet[0];
                const int a1 = offA + tet[1];
                const int a2 = offA + tet[2];
                const int a3 = offA + tet[3];

                const int b0 = offB + tet[0];
                const int b1 = offB + tet[1];
                const int b2 = offB + tet[2];
                const int b3 = offB + tet[3];

                // Triangulation of tetra×segment into 4 pentachora:
                // (a0 a1 a2 a3 b0)
                // (b0 a1 a2 a3 b1)
                // (b0 b1 a2 a3 b2)
                // (b0 b1 b2 a3 b3)
                simplices_.push_back({a0, a1, a2, a3, b0});
                simplices_.push_back({b0, a1, a2, a3, b1});
                simplices_.push_back({b0, b1, a2, a3, b2});
                simplices_.push_back({b0, b1, b2, a3, b3});
            }
        }

        // Build a cheap shadow wireframe: sample only the outer shell (ball surface) as a lat-long ring.
        // We reuse the same idea as the old nautilus surface: 2D tube around centerline.
        const int vSteps = 24;
        shadowVertsLocal_.reserve(static_cast<size_t>(tSteps) * static_cast<size_t>(vSteps));

        for (int i = 0; i < tSteps; ++i)
        {
            const float t = ringT(i);
            const Vec4f c = spiralPoint(t);

            const Vec4f c2 = spiralPoint(t + settings_.tangentDelta);
            Vec4f tan = c2 - c;
            const float tm = norm4(tan);
            if (tm > 1e-6f)
            {
                tan *= (1.0f / tm);
            }

            Vec4f n1, n2, n3;
            buildOrthonormal3(tan, n1, n2, n3);

            const float r = tubeRadius(t);

            // Simple circle in (n1,n2) plane for shadow wire
            for (int j = 0; j < vSteps; ++j)
            {
                const float a = (static_cast<float>(j) / static_cast<float>(vSteps)) * (2.0f * 3.14159265358979323846f);
                shadowVertsLocal_.push_back(c + n1 * (r * std::cos(a)) + n2 * (r * std::sin(a)));
            }
        }

        for (int i = 0; i < tSteps - 1; ++i)
        {
            for (int j = 0; j < vSteps; ++j)
            {
                const int curr = i * vSteps + j;
                const int nextV = i * vSteps + ((j + 1) % vSteps);
                const int nextT = (i + 1) * vSteps + j;

                shadowEdges_.push_back({curr, nextV});
                shadowEdges_.push_back({curr, nextT});
            }
        }
    }
};
