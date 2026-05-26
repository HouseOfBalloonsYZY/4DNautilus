#pragma once

#include "FMath.hpp"
#include "Object4D.hpp"

#include "al/graphics/al_Graphics.hpp"
#include "al/graphics/al_Mesh.hpp"
#include "al/types/al_Color.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace al;

// ----------------------------------------------------------------------------
// 4D -> viewer-local -> 3D projection (ana/kata w depth cue).
//
// One-point: uniform foreshortening of (x,y,z) from local w.
// Two-point: same w scaling on y,z; x compresses toward vanishing points at
//            +vanishX and -vanishX on the projected x-axis.
// ----------------------------------------------------------------------------

enum class ProjectionMode
{
	OnePoint = 0,
	TwoPoint = 1
};

struct ProjectionSettings
{
	ProjectionMode mode{ProjectionMode::OnePoint};

	/// Distance from origin to each x-axis vanishing point (two-point mode).
	float vanishX{24.f};
};

inline float scaleFromLocalW(float wl)
{
	if (wl < 0.f)
	{
		const float t = -wl;
		return 1.f / (1.f + t);
	}

	const float t = wl;
	return 1.f + t;
}

inline al::Vec3f projectLocal4Dto3D(const Vec4f &local, const ProjectionSettings &settings)
{
	const float s = scaleFromLocalW(local.w);
	const float y = local.y * s;
	const float z = local.z * s;

	if (settings.mode == ProjectionMode::OnePoint)
	{
		return al::Vec3f(local.x * s, y, z);
	}

	// Two-point along local x: converge to (+vanishX, 0, 0) and (-vanishX, 0, 0).
	const float x = local.x * s;
	const float vp = std::max(settings.vanishX, 1e-4f);
	const float ax = std::fabs(x);
	if (ax < 1e-8f)
	{
		return al::Vec3f(0.f, y, z);
	}

	const float xProj = x * vp / (vp + ax);
	return al::Vec3f(xProj, y, z);
}

inline al::Vec3f projectLocal4Dto3D(const Vec4f &local)
{
	return projectLocal4Dto3D(local, ProjectionSettings{});
}

inline Vec4f toViewerLocal(const Object4D &viewer, const Vec4f &world)
{
	const Rotation4D inv = viewer.rotationState.inverse();
	const Vec4f rel = world - viewer.pos;
	return inv.apply(rel);
}

inline al::Vec3f projectWorld4Dto3D(
	const Object4D &viewer,
	const Vec4f &world,
	const ProjectionSettings &settings)
{
	return projectLocal4Dto3D(toViewerLocal(viewer, world), settings);
}

inline al::Vec3f projectWorld4Dto3D(const Object4D &viewer, const Vec4f &world)
{
	return projectWorld4Dto3D(viewer, world, ProjectionSettings{});
}

inline al::Color colorFromLocalW(float wl)
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

	const al::Color &target = (wl >= 0.f) ? blueGrey : blueBright;

	al::Color c;
	c.r = white.r + t * (target.r - white.r);
	c.g = white.g + t * (target.g - white.g);
	c.b = white.b + t * (target.b - white.b);
	c.a = 1.f;
	return c;
}

// ----------------------------------------------------------------------------
// Draw helpers
// ----------------------------------------------------------------------------

inline void drawWorldAxes(
	al::Graphics &g,
	const Object4D &viewer,
	const ProjectionSettings &settings,
	float length = 4.f)
{
	al::Mesh m;
	m.primitive(al::Mesh::LINES);

	const Vec4f origin4(0.f, 0.f, 0.f, 0.f);

	const Vec4f axes4[4] = {
		Vec4f(length, 0.f, 0.f, 0.f),
		Vec4f(0.f, length, 0.f, 0.f),
		Vec4f(0.f, 0.f, length, 0.f),
		Vec4f(0.f, 0.f, 0.f, length)};

	const al::Color colors[4] = {
		al::Color(1.f, 0.f, 0.f, 1.f),
		al::Color(1.f, 1.f, 0.f, 1.f),
		al::Color(0.f, 0.4f, 1.f, 1.f),
		al::Color(0.f, 1.f, 0.f, 1.f)};

	for (int i = 0; i < 4; ++i)
	{
		const al::Vec3f p0 = projectWorld4Dto3D(viewer, origin4, settings);
		const al::Vec3f p1 = projectWorld4Dto3D(viewer, axes4[i], settings);

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

inline void drawWorldAxes(al::Graphics &g, const Object4D &viewer, float length = 4.f)
{
	drawWorldAxes(g, viewer, ProjectionSettings{}, length);
}

/// Project and draw edges defined in an object's local 4D coordinates.
inline void drawProjectedEdgesLocal(
	al::Graphics &g,
	const Object4D &viewer,
	const Object4D &object,
	const std::vector<Vec4f> &vertsLocal,
	const std::vector<std::pair<int, int>> &edges,
	const ProjectionSettings &settings,
	bool colorByLocalW = true)
{
	if (vertsLocal.empty() || edges.empty())
	{
		return;
	}

	al::Mesh mesh;
	mesh.primitive(al::Mesh::LINES);

	for (const auto &e : edges)
	{
		const Vec4f aWorld = object.pos + object.rotationState.apply(vertsLocal[e.first]);
		const Vec4f bWorld = object.pos + object.rotationState.apply(vertsLocal[e.second]);

		const Vec4f aLocal = toViewerLocal(viewer, aWorld);
		const Vec4f bLocal = toViewerLocal(viewer, bWorld);

		const al::Vec3f pa = projectLocal4Dto3D(aLocal, settings);
		const al::Vec3f pb = projectLocal4Dto3D(bLocal, settings);

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

	if (!mesh.vertices().empty())
	{
		g.meshColor();
		g.draw(mesh);
	}
}

namespace detail
{

inline float clamp01(float x)
{
	return std::max(0.f, std::min(1.f, x));
}

inline al::Color gradientColor(float t)
{
	const al::Color inner(1.f, 1.f, 1.f, 1.f);
	const al::Color outer(1.f, 100.f / 255.f, 0.f, 76.f / 255.f);

	al::Color c;
	c.r = inner.r + t * (outer.r - inner.r);
	c.g = inner.g + t * (outer.g - inner.g);
	c.b = inner.b + t * (outer.b - inner.b);
	c.a = inner.a + t * (outer.a - inner.a);
	return c;
}

} // namespace detail

/// Project and draw a nautilus tube window (geometry-only inputs).
inline void drawNautilusTubeProjected(
	al::Graphics &g,
	const Object4D &viewer,
	const Object4D &object,
	const std::vector<Vec4f> &verticesLocal,
	const std::vector<float> &hyperDists,
	int tSteps,
	int vSteps,
	int startRing,
	int ringCount,
	bool drawVertexDots,
	int pointStride,
	float pointSize,
	const ProjectionSettings &settings)
{
	if (verticesLocal.empty() || tSteps <= 0 || vSteps <= 0)
	{
		return;
	}

	const int ringCountClamped = std::max(1, std::min(ringCount, tSteps));
	const int startRingClamped = std::max(0, std::min(startRing, tSteps - 1));

	const int ringsForProj = ringCountClamped + 1;
	const size_t windowVerts = static_cast<size_t>(ringsForProj) * static_cast<size_t>(vSteps);

	std::vector<al::Vec3f> proj(windowVerts);
	std::vector<float> dists(windowVerts);

	float minD = std::numeric_limits<float>::max();
	float maxD = std::numeric_limits<float>::lowest();

	for (int k = 0; k <= ringCountClamped; ++k)
	{
		const int r = (startRingClamped + k) % tSteps;

		for (int v = 0; v < vSteps; ++v)
		{
			const int localIdx = r * vSteps + v;
			const size_t winIdx = static_cast<size_t>(k * vSteps + v);

			const Vec4f &vLoc = verticesLocal[localIdx];
			const Vec4f vWorld = object.pos + object.rotationState.apply(vLoc);

			const Vec4f vLocal = toViewerLocal(viewer, vWorld);
			proj[winIdx] = projectLocal4Dto3D(vLocal, settings);

			const float d = hyperDists[static_cast<size_t>(localIdx)];
			dists[winIdx] = d;

			minD = std::min(minD, d);
			maxD = std::max(maxD, d);
		}
	}

	if (maxD <= minD)
	{
		maxD = minD + 1.f;
	}

	g.blending(true);
	g.blendTrans();

	al::Mesh lines;
	lines.primitive(al::Mesh::LINES);

	for (int k = 0; k < ringCountClamped; ++k)
	{
		for (int v = 0; v < vSteps; ++v)
		{
			const size_t curr = static_cast<size_t>(k * vSteps + v);
			const size_t nextV = static_cast<size_t>(k * vSteps + ((v + 1) % vSteps));
			const size_t nextT = static_cast<size_t>((k + 1) * vSteps + v);

			const float d = dists[curr];
			const float t = detail::clamp01((d - minD) / (maxD - minD));
			const al::Color c = detail::gradientColor(t);

			lines.color(c);
			lines.vertex(proj[curr]);
			lines.color(c);
			lines.vertex(proj[nextV]);

			const int actualRingIndex = (startRingClamped + k) % tSteps;
			const bool isPhysicalWrap = (actualRingIndex == tSteps - 1);
			if (!isPhysicalWrap)
			{
				lines.color(c);
				lines.vertex(proj[curr]);
				lines.color(c);
				lines.vertex(proj[nextT]);
			}
		}
	}

	al::Mesh points;
	if (drawVertexDots && pointStride > 0)
	{
		points.primitive(al::Mesh::POINTS);
		g.pointSize(pointSize);

		for (int k = 0; k < ringCountClamped; k += std::max(1, pointStride))
		{
			for (int v = 0; v < vSteps; v += std::max(1, pointStride))
			{
				const size_t idx = static_cast<size_t>(k * vSteps + v);
				if (idx >= windowVerts)
				{
					continue;
				}

				const float d = dists[idx];
				const float t = detail::clamp01((d - minD) / (maxD - minD));
				const al::Color c = detail::gradientColor(t);

				points.color(c);
				points.vertex(proj[idx]);
			}
		}
	}

	if (!lines.vertices().empty())
	{
		g.meshColor();
		g.draw(lines);
	}

	if (drawVertexDots && !points.vertices().empty())
	{
		g.meshColor();
		g.draw(points);
	}

	g.blending(false);
}
