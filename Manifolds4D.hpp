#pragma once

#include <vector>

#include "al/graphics/al_Graphics.hpp"
#include "al/graphics/al_Mesh.hpp"
#include "al/types/al_Color.hpp"
#include "Fmath.hpp"
#include "object4D.hpp"

using namespace al;

// ---------------------------------------------------------------------------
// 4D -> viewer-local -> 3D projection using ana/kata style.
// local = viewerRot^{-1} * (world - viewer.pos)
// Then scale (x,y,z) by s derived from local w:
//   w_l < 0  : s in (0,1)  via  s = 1/(1 + |w_l|)
//   w_l >= 0 : s in (1,∞)  via  s = 1 +  w_l
// ---------------------------------------------------------------------------

inline Vec4f toViewerLocal(const object4D &viewer, const Vec4f &world)
{
    const Rotation4D inv = viewer.rotationState.inverse();
    Vec4f rel = world - viewer.pos;
    return inv.apply(rel);
}

inline al::Vec3f projectLocal4Dto3D(const Vec4f &local)
{
    const float wl = local.w;
    float s;
    if (wl < 0.f)
    {
        const float t = -wl;
        s = 1.f / (1.f + t); // (-∞,0) -> (0,1)
    }
    else
    {
        const float t = wl;
        s = 1.f + t; // (0,∞) -> (1,∞)
    }
    return al::Vec3f(local.x * s, local.y * s, local.z * s);
}

inline al::Vec3f projectWorld4Dto3D(const object4D &viewer, const Vec4f &world)
{
    return projectLocal4Dto3D(toViewerLocal(viewer, world));
}

inline al::Color colorFromLocalW(float wl)
{
    const float scale = 10.f;
    float t = std::fabs(wl) / scale;
    if (t > 1.f)
    {
        t = 1.f;
    }

    // Base white
    al::Color white(1.f, 1.f, 1.f, 1.f);
    // +w -> greyish blue, -w -> bright blue.
    al::Color blueGrey(0.65f, 0.72f, 0.95f, 1.f);
    al::Color blueBright(0.2f, 0.45f, 1.f, 1.f);

    const al::Color &target = (wl >= 0.f) ? blueGrey : blueBright;

    al::Color c;
    c.r = white.r + t * (target.r - white.r);
    c.g = white.g + t * (target.g - white.g);
    c.b = white.b + t * (target.b - white.b);
    c.a = 1.f;
    return c;
}

// ---------------------------------------------------------------------------
// World axes at origin: x (red), y (yellow), z (blue), w (green).
// Each drawn from (0,0,0,0) to axis * length, projected through the viewer.
// ---------------------------------------------------------------------------

inline void drawWorldAxes(al::Graphics &g, const object4D &viewer, float length = 4.f)
{
    al::Mesh m;
    m.primitive(al::Mesh::LINES);

    const Vec4f origin4(0.f, 0.f, 0.f, 0.f);

    const Vec4f axes4[4] = {
        Vec4f(length, 0.f, 0.f, 0.f), // x
        Vec4f(0.f, length, 0.f, 0.f), // y
        Vec4f(0.f, 0.f, length, 0.f), // z
        Vec4f(0.f, 0.f, 0.f, length)  // w
    };

    const al::Color colors[4] = {
        al::Color(1.f, 0.f, 0.f, 1.f),  // x red
        al::Color(1.f, 1.f, 0.f, 1.f),  // y yellow
        al::Color(0.f, 0.4f, 1.f, 1.f), // z blue
        al::Color(0.f, 1.f, 0.f, 1.f)   // w green
    };

    for (int i = 0; i < 4; ++i)
    {
        al::Vec3f p0 = projectWorld4Dto3D(viewer, origin4);
        al::Vec3f p1 = projectWorld4Dto3D(viewer, axes4[i]);

        m.color(colors[i]);
        m.vertex(p0);
        m.color(colors[i]);
        m.vertex(p1);
    }

    if (!m.vertices().empty())
    {
        g.meshColor();
        g.draw(m);
    }
}

// ---------------------------------------------------------------------------
// Simple 4D Spherinder manifold (sphere extruded along w).
// Local vertices live in object space; world position and rotation come from
// the base class object4D.
// ---------------------------------------------------------------------------

class Spherinder4D : public object4D
{
public:
    Spherinder4D()
    {
        generateGeometry();
    }

    /** Render projected edges given a viewer. */
    void drawProjected(al::Graphics &g, const object4D &viewer)
    {
        al::Mesh mesh;
        mesh.primitive(al::Mesh::LINES);

        for (const auto &e : edges_)
        {
            const Vec4f &a = vertices_[e.first];
            const Vec4f &b = vertices_[e.second];

            // Transform into world space via this object's rotation and position.
            Vec4f aWorld = pos + rotationState.apply(a);
            Vec4f bWorld = pos + rotationState.apply(b);

            // Viewer-local coordinates
            Vec4f aLocal = toViewerLocal(viewer, aWorld);
            Vec4f bLocal = toViewerLocal(viewer, bWorld);

            al::Vec3f pa = projectLocal4Dto3D(aLocal);
            al::Vec3f pb = projectLocal4Dto3D(bLocal);

            al::Color ca = colorFromLocalW(aLocal.w);
            al::Color cb = colorFromLocalW(bLocal.w);

            mesh.color(ca);
            mesh.vertex(pa);
            mesh.color(cb);
            mesh.vertex(pb);
        }

        if (mesh.vertices().size() > 0)
        {
            g.meshColor();
            g.draw(mesh);
        }
    }

private:
    std::vector<Vec4f> vertices_;
    std::vector<std::pair<int, int>> edges_;

    void generateGeometry()
    {
        vertices_.clear();
        edges_.clear();

        const int latSteps = 12;
        const int lonSteps = 24;
        const int wSteps = 6;
        const float radius = 3.0f;
        const float wLen = 6.0f;

        for (int wi = 0; wi < wSteps; ++wi)
        {
            float tW = (wSteps > 1) ? (float)wi / (float)(wSteps - 1) : 0.f;
            float wVal = -wLen + 2.f * wLen * tW;
            for (int i = 0; i <= latSteps; ++i)
            {
                float tLat = (float)i / (float)latSteps;
                float lat = -M_PI_2 + tLat * M_PI;
                for (int j = 0; j < lonSteps; ++j)
                {
                    float tLon = (float)j / (float)lonSteps;
                    float lon = tLon * M_2PI;
                    float x = radius * std::cos(lat) * std::cos(lon);
                    float y = radius * std::cos(lat) * std::sin(lon);
                    float z = radius * std::sin(lat);
                    vertices_.emplace_back(x, y, z, wVal);
                }
            }
        }

        const int sphereVerts = (latSteps + 1) * lonSteps;
        for (int wi = 0; wi < wSteps; ++wi)
        {
            int wOffset = wi * sphereVerts;
            for (int i = 0; i < latSteps; ++i)
            {
                for (int j = 0; j < lonSteps; ++j)
                {
                    int curr = wOffset + i * lonSteps + j;
                    int nextLon = wOffset + i * lonSteps + ((j + 1) % lonSteps);
                    int nextLat = wOffset + (i + 1) * lonSteps + j;
                    edges_.emplace_back(curr, nextLon);
                    edges_.emplace_back(curr, nextLat);
                    if (wi < wSteps - 1)
                    {
                        edges_.emplace_back(curr, curr + sphereVerts);
                    }
                }
            }
        }
    }
};
