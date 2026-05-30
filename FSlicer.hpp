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
// Unified dimension rule (viewer-local hyperplane w = wPlane, codimension 1):
//   4-cell  (4-simplex, 5 verts) → at most a 3D solid section   → volume mesh
//   3-simplex (4 verts, 3D flat) → at most a 2D patch in slice → surface mesh
//   2-simplex (triangle / quad boundary) → at most a 1D curve     → curve mesh
//   1-simplex (edge)               → at most a 0D point           → point mesh
//
// Analogous to slicing R^3 with a plane: volume→surface, surface→curve, curve→point.
// All geometry is supplied in world 4D; Nav4D::toLocal is the only frame change.
// ----------------------------------------------------------------------------

class Slicer4D
{
public:
	struct Settings
	{
		float wPlane{0.f};
		float sliceScale{25.f};
		float denomEps{1e-5f};
		float quant{1e-3f};

		al::Color volumeColor{0.f, 0.8f, 1.f, 0.18f};
		al::Color surfaceColor{0.f, 1.f, 0.8f, 0.35f};
		al::Color curveColor{0.f, 1.f, 0.8f, 1.f};
		al::Color pointColor{1.f, 1.f, 1.f, 0.55f};

		bool drawVolume{true};
		bool drawSurface{true};
		bool drawCurves{true};
		bool drawPoints{true};
		float pointSize{3.5f};
	};

	/// Slice output partitioned by intrinsic dimension of the cross-section.
	struct Result
	{
		al::Mesh volume;  // TRIANGLES — sections of 4-cells (3D solid in display space)
		al::Mesh surface; // TRIANGLES — sections of 3-simplices (2D patch in display space)
		al::Mesh curves;  // LINES    — sections of 2-surfaces (1D in display space)
		al::Mesh points;  // POINTS   — sections of 1-simplices
	};

	Slicer4D() = default;

	explicit Slicer4D(const Settings &settings)
		: mSettings(settings)
	{
	}

	const Settings &settings() const { return mSettings; }

	Slicer4D &settings(const Settings &s)
	{
		mSettings = s;
		return *this;
	}

	void clearGeometry()
	{
		mVertsWorld.clear();
		mCells4.clear();
		mSimplices3.clear();
		mQuads2.clear();
		mTriangles2.clear();
		mEdges1.clear();
	}

	void setVerticesWorld(const std::vector<Vec4f> &vertsWorld)
	{
		mVertsWorld = vertsWorld;
	}

	void add4Cell(const std::array<int, 5> &indices) { mCells4.push_back(indices); }

	/// Filled tetrahedron in 4D (3-simplex); slice is at most a 2D region.
	void add3Simplex(const std::array<int, 4> &indices) { mSimplices3.push_back(indices); }

	/// 2-surface quad (4 boundary vertices); slice is at most a 1D curve.
	void add2Quad(const std::array<int, 4> &indices) { mQuads2.push_back(indices); }

	/// 2-simplex triangle; slice is at most a segment.
	void add2Triangle(const std::array<int, 3> &indices) { mTriangles2.push_back(indices); }

	void add1Edge(int a, int b) { mEdges1.emplace_back(a, b); }

	Result slice(const Nav4D &viewer) const
	{
		Result out;
		out.volume.primitive(al::Mesh::TRIANGLES);
		out.surface.primitive(al::Mesh::TRIANGLES);
		out.curves.primitive(al::Mesh::LINES);
		out.points.primitive(al::Mesh::POINTS);

		if (mVertsWorld.empty())
		{
			return out;
		}

		std::vector<Vec4f> vertsLocal(mVertsWorld.size());
		for (size_t i = 0; i < mVertsWorld.size(); ++i)
		{
			vertsLocal[i] = viewer.toLocal(mVertsWorld[i]);
		}

		std::vector<Tri3> volumeTris;
		volumeTris.reserve(mCells4.size() * 6);

		for (const auto &cell : mCells4)
		{
			slice4Cell(vertsLocal, cell, volumeTris);
		}

		emitVolumeMesh(out.volume, volumeTris);

		for (const auto &simp : mSimplices3)
		{
			slice3Simplex(vertsLocal, simp, out.surface);
		}

		for (const auto &quad : mQuads2)
		{
			slice2Quad(vertsLocal, quad, out.curves, out.points);
		}

		for (const auto &tri : mTriangles2)
		{
			slice2Triangle(vertsLocal, tri, out.curves, out.points);
		}

		for (const auto &edge : mEdges1)
		{
			slice1Edge(vertsLocal, edge, out.points);
		}

		return out;
	}

	void draw(al::Graphics &g, const Result &result) const
	{
		g.depthTesting(true);

		if (mSettings.drawVolume && !result.volume.vertices().empty())
		{
			g.blending(true);
			g.blendTrans();
			g.meshColor();
			g.draw(result.volume);
			g.blending(false);
		}

		if (mSettings.drawSurface && !result.surface.vertices().empty())
		{
			g.blending(true);
			g.blendTrans();
			g.meshColor();
			g.draw(result.surface);
			g.blending(false);
		}

		if (mSettings.drawCurves && !result.curves.vertices().empty())
		{
			g.meshColor();
			g.draw(result.curves);
		}

		if (mSettings.drawPoints && !result.points.vertices().empty())
		{
			g.pointSize(mSettings.pointSize);
			g.meshColor();
			g.draw(result.points);
		}
	}

private:
	struct Tri3
	{
		al::Vec3f a, b, c;
	};

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
			{
				return static_cast<uint32_t>(v) * 2654435761u;
			};
			size_t r = h(k.a.x) ^ (h(k.a.y) << 1) ^ (h(k.a.z) << 2);
			r ^= (h(k.b.x) << 3) ^ (h(k.b.y) << 4) ^ (h(k.b.z) << 5);
			r ^= (h(k.c.x) << 6) ^ (h(k.c.y) << 7) ^ (h(k.c.z) << 8);
			return r;
		}
	};

	Settings mSettings{};
	std::vector<Vec4f> mVertsWorld;
	std::vector<std::array<int, 5>> mCells4;
	std::vector<std::array<int, 4>> mSimplices3;
	std::vector<std::array<int, 4>> mQuads2;
	std::vector<std::array<int, 3>> mTriangles2;
	std::vector<std::pair<int, int>> mEdges1;

	static bool edgeCrossesWPlane(float w0, float w1, float wPlane)
	{
		return (w0 <= wPlane && w1 >= wPlane) || (w0 >= wPlane && w1 <= wPlane);
	}

	static al::Vec3f local4DToSlice3D(const Vec4f &pLocal, float sliceScale)
	{
		return al::Vec3f(pLocal.x * sliceScale, pLocal.y * sliceScale, pLocal.z * sliceScale);
	}

	bool intersectEdge(
		const Vec4f &a,
		const Vec4f &b,
		Vec4f &out4,
		al::Vec3f &out3) const
	{
		if (!edgeCrossesWPlane(a.w, b.w, mSettings.wPlane))
		{
			return false;
		}

		const float denom = b.w - a.w;
		if (std::fabs(denom) <= mSettings.denomEps)
		{
			return false;
		}

		const float t = (mSettings.wPlane - a.w) / denom;
		if (t < 0.f || t > 1.f)
		{
			return false;
		}

		out4 = a + (b - a) * t;
		out3 = local4DToSlice3D(out4, mSettings.sliceScale);
		return true;
	}

	static QuantKey3 quantize3(const al::Vec3f &p, float q)
	{
		const float inv = 1.f / q;
		return QuantKey3{
			static_cast<int32_t>(std::lround(p.x * inv)),
			static_cast<int32_t>(std::lround(p.y * inv)),
			static_cast<int32_t>(std::lround(p.z * inv))};
	}

	static QuantKeyTri makeTriKey(const al::Vec3f &p0, const al::Vec3f &p1, const al::Vec3f &p2, float q)
	{
		QuantKey3 k0 = quantize3(p0, q);
		QuantKey3 k1 = quantize3(p1, q);
		QuantKey3 k2 = quantize3(p2, q);

		auto less = [](const QuantKey3 &a, const QuantKey3 &b)
		{
			if (a.x != b.x)
			{
				return a.x < b.x;
			}
			if (a.y != b.y)
			{
				return a.y < b.y;
			}
			return a.z < b.z;
		};

		QuantKey3 a = k0, b = k1, c = k2;
		if (less(b, a))
		{
			std::swap(a, b);
		}
		if (less(c, b))
		{
			std::swap(b, c);
		}
		if (less(b, a))
		{
			std::swap(a, b);
		}

		return QuantKeyTri{a, b, c};
	}

	void dedupePoints(const std::vector<al::Vec3f> &in, std::vector<al::Vec3f> &out) const
	{
		out.clear();
		std::unordered_map<uint64_t, int> seen;
		seen.reserve(in.size() * 2);

		for (const al::Vec3f &p : in)
		{
			const QuantKey3 qk = quantize3(p, mSettings.quant);
			const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(qk.x)) << 42)
				^ (static_cast<uint64_t>(static_cast<uint32_t>(qk.y)) << 21)
				^ static_cast<uint64_t>(static_cast<uint32_t>(qk.z));

			if (seen.find(key) == seen.end())
			{
				seen[key] = static_cast<int>(out.size());
				out.push_back(p);
			}
		}
	}

	void triangulateConvexFan(
		const std::vector<al::Vec3f> &pts,
		std::vector<Tri3> &outTris) const
	{
		if (pts.size() < 3)
		{
			return;
		}

		for (size_t i = 1; i + 1 < pts.size(); ++i)
		{
			outTris.push_back(Tri3{pts[0], pts[i], pts[i + 1]});
		}
	}

	void slice4Cell(
		const std::vector<Vec4f> &vertsLocal,
		const std::array<int, 5> &cell,
		std::vector<Tri3> &outTris) const
	{
		static const int ea[10] = {0, 0, 0, 0, 1, 1, 1, 2, 2, 3};
		static const int eb[10] = {1, 2, 3, 4, 2, 3, 4, 3, 4, 4};

		const Vec4f p4[5] = {
			vertsLocal[static_cast<size_t>(cell[0])],
			vertsLocal[static_cast<size_t>(cell[1])],
			vertsLocal[static_cast<size_t>(cell[2])],
			vertsLocal[static_cast<size_t>(cell[3])],
			vertsLocal[static_cast<size_t>(cell[4])]};

		std::vector<al::Vec3f> raw;
		raw.reserve(10);

		for (int e = 0; e < 10; ++e)
		{
			Vec4f q4;
			al::Vec3f q3;
			if (intersectEdge(p4[ea[e]], p4[eb[e]], q4, q3))
			{
				raw.push_back(q3);
			}
		}

		std::vector<al::Vec3f> pts;
		dedupePoints(raw, pts);

		if (pts.size() < 4)
		{
			return;
		}

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
					if (nrm.mag() < eps)
					{
						continue;
					}

					int posSide = 0;
					int negSide = 0;
					for (int m = 0; m < n; ++m)
					{
						if (m == i || m == j || m == k)
						{
							continue;
						}
						const float d = al::dot(nrm, pts[m] - a);
						if (d > eps)
						{
							posSide++;
						}
						else if (d < -eps)
						{
							negSide++;
						}
					}

					if (posSide && negSide)
					{
						continue;
					}

					outTris.push_back(Tri3{a, b, c});
				}
			}
		}
	}

	void emitVolumeMesh(al::Mesh &mesh, const std::vector<Tri3> &tris) const
	{
		std::unordered_map<QuantKeyTri, int, QuantKeyTriHash> triCounts;
		triCounts.reserve(tris.size() * 2);

		for (const Tri3 &t : tris)
		{
			const QuantKeyTri key = makeTriKey(t.a, t.b, t.c, mSettings.quant);
			triCounts[key] += 1;
		}

		for (const Tri3 &t : tris)
		{
			const QuantKeyTri key = makeTriKey(t.a, t.b, t.c, mSettings.quant);
			if (triCounts[key] != 1)
			{
				continue;
			}

			mesh.color(mSettings.volumeColor);
			mesh.vertex(t.a);
			mesh.color(mSettings.volumeColor);
			mesh.vertex(t.b);
			mesh.color(mSettings.volumeColor);
			mesh.vertex(t.c);
		}
	}

	void slice3Simplex(
		const std::vector<Vec4f> &vertsLocal,
		const std::array<int, 4> &simp,
		al::Mesh &surface) const
	{
		static const int ea[6] = {0, 0, 0, 1, 1, 2};
		static const int eb[6] = {1, 2, 3, 2, 3, 3};

		const Vec4f p4[4] = {
			vertsLocal[static_cast<size_t>(simp[0])],
			vertsLocal[static_cast<size_t>(simp[1])],
			vertsLocal[static_cast<size_t>(simp[2])],
			vertsLocal[static_cast<size_t>(simp[3])]};

		std::vector<al::Vec3f> raw;
		raw.reserve(6);

		for (int e = 0; e < 6; ++e)
		{
			Vec4f q4;
			al::Vec3f q3;
			if (intersectEdge(p4[ea[e]], p4[eb[e]], q4, q3))
			{
				raw.push_back(q3);
			}
		}

		std::vector<al::Vec3f> pts;
		dedupePoints(raw, pts);

		if (pts.size() < 3)
		{
			return;
		}

		std::vector<Tri3> tris;
		triangulateConvexFan(pts, tris);

		for (const Tri3 &t : tris)
		{
			surface.color(mSettings.surfaceColor);
			surface.vertex(t.a);
			surface.color(mSettings.surfaceColor);
			surface.vertex(t.b);
			surface.color(mSettings.surfaceColor);
			surface.vertex(t.c);
		}
	}

	void slice2Quad(
		const std::vector<Vec4f> &vertsLocal,
		const std::array<int, 4> &quad,
		al::Mesh &curves,
		al::Mesh &points) const
	{
		static const int ea[4] = {0, 1, 2, 3};
		static const int eb[4] = {1, 2, 3, 0};

		const Vec4f p4[4] = {
			vertsLocal[static_cast<size_t>(quad[0])],
			vertsLocal[static_cast<size_t>(quad[1])],
			vertsLocal[static_cast<size_t>(quad[2])],
			vertsLocal[static_cast<size_t>(quad[3])]};

		al::Vec3f isect[4];
		int isectCount = 0;

		for (int e = 0; e < 4; ++e)
		{
			Vec4f q4;
			al::Vec3f q3;
			if (intersectEdge(p4[ea[e]], p4[eb[e]], q4, q3))
			{
				isect[isectCount++] = q3;
				if (isectCount == 4)
				{
					break;
				}
			}
		}

		auto emitSegment = [&](const al::Vec3f &a, const al::Vec3f &b)
		{
			curves.color(mSettings.curveColor);
			curves.vertex(a);
			curves.color(mSettings.curveColor);
			curves.vertex(b);

			if (mSettings.drawPoints)
			{
				points.color(mSettings.pointColor);
				points.vertex(a);
				points.color(mSettings.pointColor);
				points.vertex(b);
			}
		};

		if (isectCount == 2)
		{
			emitSegment(isect[0], isect[1]);
		}
		else if (isectCount == 4)
		{
			emitSegment(isect[0], isect[1]);
			emitSegment(isect[2], isect[3]);
		}
	}

	void slice2Triangle(
		const std::vector<Vec4f> &vertsLocal,
		const std::array<int, 3> &tri,
		al::Mesh &curves,
		al::Mesh &points) const
	{
		static const int ea[3] = {0, 1, 2};
		static const int eb[3] = {1, 2, 0};

		const Vec4f p4[3] = {
			vertsLocal[static_cast<size_t>(tri[0])],
			vertsLocal[static_cast<size_t>(tri[1])],
			vertsLocal[static_cast<size_t>(tri[2])]};

		al::Vec3f isect[3];
		int isectCount = 0;

		for (int e = 0; e < 3; ++e)
		{
			Vec4f q4;
			al::Vec3f q3;
			if (intersectEdge(p4[ea[e]], p4[eb[e]], q4, q3))
			{
				isect[isectCount++] = q3;
			}
		}

		if (isectCount == 2)
		{
			curves.color(mSettings.curveColor);
			curves.vertex(isect[0]);
			curves.color(mSettings.curveColor);
			curves.vertex(isect[1]);

			if (mSettings.drawPoints)
			{
				points.color(mSettings.pointColor);
				points.vertex(isect[0]);
				points.color(mSettings.pointColor);
				points.vertex(isect[1]);
			}
		}
	}

	void slice1Edge(
		const std::vector<Vec4f> &vertsLocal,
		const std::pair<int, int> &edge,
		al::Mesh &points) const
	{
		Vec4f q4;
		al::Vec3f q3;
		if (intersectEdge(
				vertsLocal[static_cast<size_t>(edge.first)],
				vertsLocal[static_cast<size_t>(edge.second)],
				q4,
				q3))
		{
			points.color(mSettings.pointColor);
			points.vertex(q3);
		}
	}
};

// --- Legacy types / shims (map onto Slicer4D) ---

using SliceSettings = Slicer4D::Settings;
using HyperSliceSettings = Slicer4D::Settings;

struct SliceResult
{
	al::Mesh segments;
	al::Mesh points;
};

struct HyperSliceResult
{
	al::Mesh triangles;
	al::Mesh edges;
	al::Mesh points;
};

inline SliceResult sliceQuadsViewerLocal(
	const Nav4D &viewer,
	const std::vector<Vec4f> &vertsWorld,
	const std::vector<std::array<int, 4>> &quads,
	const SliceSettings &settings)
{
	Slicer4D slicer(settings);
	slicer.setVerticesWorld(vertsWorld);
	for (const auto &q : quads)
	{
		slicer.add2Quad(q);
	}

	const Slicer4D::Result r = slicer.slice(viewer);
	SliceResult out;
	out.segments = r.curves;
	out.points = r.points;
	return out;
}

inline HyperSliceResult slice4SimplicesViewerLocal(
	const Nav4D &viewer,
	const std::vector<Vec4f> &vertsWorld,
	const std::vector<std::array<int, 5>> &simplices,
	const HyperSliceSettings &settings)
{
	Slicer4D slicer(settings);
	slicer.setVerticesWorld(vertsWorld);
	for (const auto &s : simplices)
	{
		slicer.add4Cell(s);
	}

	const Slicer4D::Result r = slicer.slice(viewer);

	HyperSliceResult out;
	out.triangles = r.volume;
	out.points = r.points;

	if (settings.drawVolume && !r.volume.vertices().empty())
	{
		out.edges.primitive(al::Mesh::LINES);
		const auto &v = r.volume.vertices();
		for (size_t i = 0; i + 2 < v.size(); i += 3)
		{
			out.edges.color(settings.volumeColor);
			out.edges.vertex(v[i + 0]);
			out.edges.color(settings.volumeColor);
			out.edges.vertex(v[i + 1]);

			out.edges.color(settings.volumeColor);
			out.edges.vertex(v[i + 1]);
			out.edges.color(settings.volumeColor);
			out.edges.vertex(v[i + 2]);

			out.edges.color(settings.volumeColor);
			out.edges.vertex(v[i + 2]);
			out.edges.color(settings.volumeColor);
			out.edges.vertex(v[i + 0]);
		}
	}

	return out;
}

inline void drawSliceResult(al::Graphics &g, const SliceResult &r, const SliceSettings &settings)
{
	Slicer4D::Result bundle;
	bundle.curves = r.segments;
	bundle.points = r.points;
	Slicer4D(settings).draw(g, bundle);
}

inline void drawHyperSlice(al::Graphics &g, const HyperSliceResult &r, const HyperSliceSettings &settings)
{
	Slicer4D::Result bundle;
	bundle.volume = r.triangles;
	bundle.points = r.points;

	if (settings.drawVolume && !r.triangles.vertices().empty())
	{
		bundle.curves = r.edges;
	}

	Slicer4D(settings).draw(g, bundle);
}
