#pragma once

#include "Nav4D.hpp"

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
// Visibility (Nav4D local, non-360 camera):
//   Keep points with local z <= 0 (in front; viewer looks along -z) and
//   local w <= 0 (ana only). Discard local z > 0 (behind) and w > 0 (kata).
//
// One-point: uniform foreshortening of (x,y,z) from local w (ana branch only).
// Two-point: same w scaling on y,z; x compresses toward vanishing points at
//            +vanishX and -vanishX on the projected x-axis.
// ----------------------------------------------------------------------------

enum class Perspective
{
	OnePoint = 0,
	TwoPoint = 1
};

struct ProjectionSettings
{
	Perspective mode{Perspective::TwoPoint};

	/// Distance from origin to each x-axis vanishing point (two-point mode).
	float vanishX{100000.f};
};

/// 4D→3D projection with Nav4D half-space culling (front + ana only).
class ProjectorPlane
{
public:
	ProjectorPlane() = default;

	explicit ProjectorPlane(const ProjectionSettings &settings)
		: mSettings(settings)
	{
	}

	const ProjectionSettings &settings() const { return mSettings; }

	ProjectorPlane &settings(const ProjectionSettings &s)
	{
		mSettings = s;
		return *this;
	}

	/// Object-local 4D → world 4D (any Object4D pose).
	static Vec4f objectToWorld(const Object4D &object, const Vec4f &local)
	{
		return object.pos + object.rotationState.apply(local);
	}

	/// True when the point lies in the visible half-spaces (local z <= 0, local w <= 0).
	static bool isProjectableLocal(const Vec4f &local)
	{
		return local.z <= 0.f && local.w <= 0.f;
	}

	/// Project viewer-local 4D if projectable; returns false when culled (behind or kata).
	bool tryProjectLocal(const Vec4f &local, Vec3f &out) const
	{
		if (!isProjectableLocal(local))
		{
			return false;
		}
		out = projectLocalVisible(local);
		return true;
	}

	/// Project a point already in viewer-local 4D (caller must ensure isProjectableLocal).
	Vec3f projectLocal(const Vec4f &local) const
	{
		return projectLocalVisible(local);
	}

	/// Depth cue from viewer-local w: ana (-w) → blue-bright (kata branch unused after cull).
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

	/// World 4D → viewer-local → projected 3D (culled when behind or kata).
	bool tryProjectWorld(const Nav4D &viewer, const Vec4f &world, Vec3f &out) const
	{
		return tryProjectLocal(viewer.toLocal(world), out);
	}

	/// World 4D → projected 3D; only defined when tryProjectWorld succeeds.
	Vec3f projectWorld(const Nav4D &viewer, const Vec4f &world) const
	{
		return projectLocal(viewer.toLocal(world));
	}

	/// Batch: world vertices → projected 3D (culled vertices omitted; order not index-stable).
	std::vector<Vec3f> projectWorldVertices(
		const Nav4D &viewer,
		const std::vector<Vec4f> &vertsWorld) const
	{
		std::vector<Vec3f> out;
		out.reserve(vertsWorld.size());
		for (const Vec4f &w : vertsWorld)
		{
			Vec3f p;
			if (tryProjectWorld(viewer, w, p))
			{
				out.push_back(p);
			}
		}
		return out;
	}

	/// Indexed line soup in world 4D → projected line mesh (edges with culled endpoints skipped).
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

			Vec3f pa;
			Vec3f pb;
			if (!tryProjectLocal(aLocal, pa) || !tryProjectLocal(bLocal, pb))
			{
				continue;
			}

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
			Vec4f(0.f, 0.f, 0.f, length)}; // +w = kata (culled)

		const al::Color colors[4] = {
			al::Color(1.f, 0.f, 0.f, 1.f),
			al::Color(1.f, 1.f, 0.f, 1.f),
			al::Color(0.f, 0.4f, 1.f, 1.f),
			al::Color(0.f, 1.f, 0.f, 1.f)};

		Vec3f p0;
		if (!tryProjectWorld(viewer, origin4, p0))
		{
			return;
		}

		for (int i = 0; i < 4; ++i)
		{
			Vec3f p1;
			if (!tryProjectWorld(viewer, axes4[i], p1))
			{
				continue;
			}

			m.color(colors[i]);
			m.vertex(p0);
			m.color(colors[i]);
			m.vertex(p1);
		}

		drawLineMesh(g, m);
	}

private:
	ProjectionSettings mSettings{};

	/// Project viewer-local 4D that is already known projectable (z <= 0, w <= 0).
	Vec3f projectLocalVisible(const Vec4f &local) const
	{
		const float s = scaleFromLocalW(local.w);
		const float y = local.y * s;
		const float z = local.z * s;

		if (mSettings.mode == Perspective::OnePoint)
		{
			return Vec3f(local.x * s, y, z);
		}

		const float x = local.x * s;
		const float vp = std::max(mSettings.vanishX, 1e-4f);
		const float ax = std::fabs(x);
		if (ax < 1e-8f)
		{
			return Vec3f(0.f, y, z);
		}

		const float xProj = x * vp / (vp + ax);
		return Vec3f(xProj, y, z);
	}

	/// Foreshortening from viewer-local w (ana only after cull: w <= 0).
	static float scaleFromLocalW(float wl)
	{
		const float t = -wl; // wl <= 0 on visible points
		return 1.f / (1.f + t);
	}
};
