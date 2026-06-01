#pragma once

#include "Nav4D.hpp"
#include "FProjectorPlane.hpp"

#include "al/graphics/al_Graphics.hpp"
#include "al/graphics/al_Mesh.hpp"
#include "al/types/al_Color.hpp"

#include <cmath>
#include <utility>
#include <vector>

using namespace al;

// ----------------------------------------------------------------------------
// 4D viewer-local -> 3D projection (ana/kata w depth cue).
//
// W-axis convention (viewer-local, project-wide):
//   kata = +w
//   ana  = -w
//
// One-point: uniform foreshortening of (x,y,z) from local w.
// Two-point: same w scaling on y,z; x compresses toward vanishing points at
//            +vanishX and -vanishX on the projected x-axis.
// ----------------------------------------------------------------------------


/// Universal 4D→3D projection layer. Operates on arbitrary world geometry via Nav4D::toLocal.
class Projection4D
{
private:
	ProjectionSettings mSettings{};

	/// Foreshortening from viewer-local w: ana (-w) shrinks, kata (+w) grows.
	static float scaleFromLocalW(float wl)
	{
		if (wl < 0.f) // ana
		{
			const float t = -wl;
			return 1.f / (1.f + t);
		}

		// kata (+w)
		const float t = wl;
		return 1.f + t;
	}

public:
	Projection4D() = default;

	explicit Projection4D(const ProjectionSettings &settings)
		: mSettings(settings)
	{
	}

	const ProjectionSettings &settings() const { return mSettings; }

	Projection4D &settings(const ProjectionSettings &s)
	{
		mSettings = s;
		return *this;
	}

	/// Object-local 4D → world 4D (any Object4D pose).
	static Vec4f objectToWorld(const Object4D &object, const Vec4f &local)
	{
		return object.pos + object.rotationState.apply(local);
	}

	/// Project a point already in viewer-local 4D.
	Vec3f projectLocal(const Vec4f &local) const
	{
		const float s = scaleFromLocalW(local.w);
        const float x = local.x * s;
		const float y = local.y * s;
		const float z = local.z * s;

        // one-point perspective: uniform scaling of x with w
		if (mSettings.mode == Perspective::OnePoint)
		{
			return Vec3f(x, y, z);
		}

        // two-point perspective: x compresses toward vanishing points at ±vanishX
        const float eps = 1e-7f; // super small non-zero threshold for negligible angle
		const float vanishPoint = std::max(mSettings.vanishX, eps);
		const float xAbsolute = std::fabs(x);
		if (xAbsolute < eps)
		{
			return Vec3f(0.f, y, z);
		}
		const float xTwoPoints = x * vanishPoint / (vanishPoint + xAbsolute);
		return Vec3f(xTwoPoints, y, z);
	}

	/// Depth cue from viewer-local w: kata (+w) → blue-grey, ana (-w) → blue-bright.
	al::Color colorFromLocalW(float wl) const
	{
		const float scale = 10.f;
		float t = std::fabs(wl) / scale;
		if (t > 1.f)
		{
			t = 1.f;
		}

		const al::Color white(1.f, 1.f, 1.f, 1.f);
		const al::Color blueGrey(0.65f, 0.72f, 0.95f, 1.f);
		const al::Color blueBright(0.2f, 0.45f, 1.f, 1.f);

		const al::Color &target = (wl >= 0.f) ? blueGrey : blueBright; // +w kata : -w ana

		al::Color c;
		c.r = white.r + t * (target.r - white.r);
		c.g = white.g + t * (target.g - white.g);
		c.b = white.b + t * (target.b - white.b);
		c.a = 1.f;
		return c;
	}

	/// World 4D → viewer-local → projected 3D.
	Vec3f projectWorld(const Nav4D &viewer, const Vec4f &world) const
	{
		return projectLocal(viewer.toLocal(world));
	}

	/// Batch: world vertices → projected 3D (same order as input).
	std::vector<Vec3f> projectWorldVertices(
		const Nav4D &viewer,
		const std::vector<Vec4f> &vertsWorld) const
	{
		std::vector<Vec3f> out;
		out.reserve(vertsWorld.size());
		for (const Vec4f &w : vertsWorld)
		{
			out.push_back(projectWorld(viewer, w));
		}
		return out;
	}

	/// Indexed line soup in world 4D → projected line mesh data.
	al::Mesh buildProjectedLineMesh(
		const Nav4D &viewer,
		const std::vector<Vec4f> &vertsWorld,
		const std::vector<std::pair<int, int>> &edges,
		bool colorByLocalW = true) const
	{
		al::Mesh mesh;
		mesh.primitive(al::Mesh::LINES);

		if (vertsWorld.empty() || edges.empty())
		{
			return mesh;
		}

		for (const auto &e : edges)
		{
			const Vec4f aLocal = viewer.toLocal(vertsWorld[e.first]);
			const Vec4f bLocal = viewer.toLocal(vertsWorld[e.second]);
			const Vec3f pa = projectLocal(aLocal);
			const Vec3f pb = projectLocal(bLocal);

			if (colorByLocalW)
			{
				mesh.color(colorFromLocalW(aLocal.w));
				mesh.vertex(pa);
				mesh.color(colorFromLocalW(bLocal.w));
				mesh.vertex(pb);
			}
			else
			{
				mesh.vertex(pa);
				mesh.vertex(pb);
			}
		}

		return mesh;
	}

	/// Object-local vertices + pose → projected line mesh.
	al::Mesh buildProjectedLineMeshFromObjectLocal(
		const Nav4D &viewer,
		const Object4D &object,
		const std::vector<Vec4f> &vertsLocal,
		const std::vector<std::pair<int, int>> &edges,
		bool colorByLocalW = true) const
	{
		std::vector<Vec4f> vertsWorld;
		vertsWorld.reserve(vertsLocal.size());
		for (const Vec4f &vLoc : vertsLocal)
		{
			vertsWorld.push_back(objectToWorld(object, vLoc));
		}
		return buildProjectedLineMesh(viewer, vertsWorld, edges, colorByLocalW);
	}

	void drawLineMesh(al::Graphics &g, const al::Mesh &lines) const
	{
		if (lines.vertices().empty())
		{
			return;
		}
		g.meshColor();
		g.draw(lines);
	}

	void drawWorldAxes(al::Graphics &g, const Nav4D &viewer, float length = 4.f) const
	{
		al::Mesh m;
		m.primitive(al::Mesh::LINES);

		const Vec4f origin4(0.f, 0.f, 0.f, 0.f);
		const Vec4f axes4[4] = {
			Vec4f(length, 0.f, 0.f, 0.f),
			Vec4f(0.f, length, 0.f, 0.f),
			Vec4f(0.f, 0.f, length, 0.f),
			Vec4f(0.f, 0.f, 0.f, length)}; // +w = kata

		const al::Color colors[4] = {
			al::Color(1.f, 0.f, 0.f, 1.f),
			al::Color(1.f, 1.f, 0.f, 1.f),
			al::Color(0.f, 0.4f, 1.f, 1.f),
			al::Color(0.f, 1.f, 0.f, 1.f)};

		for (int i = 0; i < 4; ++i)
		{
			const Vec3f p0 = projectWorld(viewer, origin4);
			const Vec3f p1 = projectWorld(viewer, axes4[i]);
			m.color(colors[i]);
			m.vertex(p0);
			m.color(colors[i]);
			m.vertex(p1);
		}

		drawLineMesh(g, m);
	}


};