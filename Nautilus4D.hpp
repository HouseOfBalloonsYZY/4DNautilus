#pragma once

#include "Manifolds4D.hpp"

#include "al/io/al_Imgui.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace w4d
{

// ----------------------------------------------------------------------------
// 4D Nautilus / Spiral Shell
// Translation of Processing `FourDSpiralShell.pde` geometry into Allolib.
//
// Geometry model:
// - For each ring parameter t:
//   - center(t) = 4D logarithmic spiral point
//   - tangent = normalize(center(t + delta) - center(t))
//   - build two orthonormal vectors (n1, n2) orthogonal to tangent
//   - extrude a growing "tube" hyper-circle around center(t) in the span(n1,n2)
// ----------------------------------------------------------------------------
class Nautilus4D : public object4D
{
public:
	Nautilus4D()
	{
		generateGeometry();
	}

	// Update spiral angular multipliers; regenerates geometry if they change.
	void updateMultipliers(float nm1, float nm2, float nm3)
	{
		const float eps = 1e-3f;
		if (std::fabs(m1_ - nm1) > eps || std::fabs(m2_ - nm2) > eps || std::fabs(m3_ - nm3) > eps)
		{
			m1_ = nm1;
			m2_ = nm2;
			m3_ = nm3;
			generateGeometry();
		}
	}

	// Draw either full nautilus or a selective window (matches Processing
	// `renderWindow(startStep, visibleSteps)` semantics).
	void drawProjected(al::Graphics& g, const object4D& viewer)
	{
		const int ringCount = selectiveDisplay_
			? std::max(1, std::min(visibleRings_, tSteps_))
			: tSteps_;

		const int startRing = std::max(0, std::min(startRing_, tSteps_ - 1));
		drawProjectedWindow(g, viewer, startRing, ringCount);
	}

	// Export a quad soup for the currently displayed ring window.
	// This is intentionally "data only" so slicing code can remain decoupled.
	void buildWorldQuadSoup(std::vector<Vec4f>& vertsWorld, std::vector<std::array<int, 4>>& quads) const
	{
		const int ringCount = selectiveDisplay_
			? std::max(1, std::min(visibleRings_, tSteps_))
			: tSteps_;
		const int startRing = std::max(0, std::min(startRing_, tSteps_ - 1));

		buildWorldQuadSoupWindow(vertsWorld, quads, startRing, ringCount);
	}

	// Export centerline samples (world-space) and radius per ring for implicit-volume slicing.
	void buildWorldCenterlineSamples(std::vector<Vec4f>& centersWorld, std::vector<float>& radii) const
	{
		const int ringCount = selectiveDisplay_
			? std::max(1, std::min(visibleRings_, tSteps_))
			: tSteps_;
		const int startRing = std::max(0, std::min(startRing_, tSteps_ - 1));

		buildWorldCenterlineSamplesWindow(centersWorld, radii, startRing, ringCount);
	}

	void buildWorldCenterlineSamplesWindow(
		std::vector<Vec4f>& centersWorld,
		std::vector<float>& radii,
		int startRing,
		int ringCount) const
	{
		centersWorld.clear();
		radii.clear();

		if (tSteps_ <= 0)
		{
			return;
		}

		const int ringCountClamped = std::max(1, std::min(ringCount, tSteps_));
		const int startRingClamped = std::max(0, std::min(startRing, tSteps_ - 1));

		centersWorld.reserve(static_cast<size_t>(ringCountClamped));
		radii.reserve(static_cast<size_t>(ringCountClamped));

		for (int k = 0; k < ringCountClamped; ++k)
		{
			const int ringIdx = (startRingClamped + k) % tSteps_;
			const float t = ringParamT(ringIdx);
			const Vec4f centerLocal = getSpiralPoint(t);
			const float rTube = (a_ * std::exp(tubeGrow_ * t)) * tubeScale_;

			const Vec4f centerWorld = pos + rotationState_.apply(centerLocal);
			centersWorld.push_back(centerWorld);
			radii.push_back(rTube);
		}
	}

	void buildWorldQuadSoupWindow(
		std::vector<Vec4f>& vertsWorld,
		std::vector<std::array<int, 4>>& quads,
		int startRing,
		int ringCount) const
	{
		vertsWorld.clear();
		quads.clear();

		if (vertices_.empty() || tSteps_ <= 1 || vSteps_ <= 2)
		{
			return;
		}

		const int ringCountClamped = std::max(1, std::min(ringCount, tSteps_));
		const int startRingClamped = std::max(0, std::min(startRing, tSteps_ - 1));

		// Like Processing: renderWindow(startRing, ringCount) samples ringCount+1 rings.
		const int ringsForSoup = ringCountClamped + 1;
		const int vertsPerRing = vSteps_;
		const size_t totalVerts = static_cast<size_t>(ringsForSoup) * static_cast<size_t>(vertsPerRing);
		vertsWorld.reserve(totalVerts);

		// Emit world vertices in (k,v) order for this window.
		for (int k = 0; k <= ringCountClamped; ++k)
		{
			const int r = (startRingClamped + k) % tSteps_;
			for (int v = 0; v < vSteps_; ++v)
			{
				const int localIdx = r * vSteps_ + v;
				const Vec4f vWorld = pos + rotationState_.apply(vertices_[localIdx]);
				vertsWorld.push_back(vWorld);
			}
		}

		// Build quads between ring k and ring k+1, around v.
		// Skip longitudinal stitching across the "physical end" (same condition as draw).
		quads.reserve(static_cast<size_t>(ringCountClamped) * static_cast<size_t>(vSteps_));
		for (int k = 0; k < ringCountClamped; ++k)
		{
			const int actualRingIndex = (startRingClamped + k) % tSteps_;
			const bool isPhysicalWrap = (actualRingIndex == tSteps_ - 1);
			if (isPhysicalWrap)
			{
				continue;
			}

			for (int v = 0; v < vSteps_; ++v)
			{
				const int vNext = (v + 1) % vSteps_;

				const int a = k * vSteps_ + v;
				const int b = k * vSteps_ + vNext;
				const int c = (k + 1) * vSteps_ + vNext;
				const int d = (k + 1) * vSteps_ + v;

				quads.push_back({a, b, c, d});
			}
		}
	}

	// Minimal ImGui controls to mirror the Processing UI.
	void drawImGuiControls()
	{
		ImGui::Separator();
		ImGui::Text("4D Nautilus controls");

		ImGui::Checkbox("Selective display", &selectiveDisplay_);

		if (selectiveDisplay_)
		{
			ImGui::SliderInt("Start ring", &startRing_, 0, tSteps_ - 1);
			ImGui::SliderInt("Visible rings", &visibleRings_, 50, std::min(1000, tSteps_));
		}

		bool regen = false;

		regen |= ImGui::SliderFloat("m1 (GR)", &m1_, 0.1f, 10.0f, "%.3f");
		regen |= ImGui::SliderFloat("m2 (E)", &m2_, 0.1f, 10.0f, "%.3f");
		regen |= ImGui::SliderFloat("m3 (PI)", &m3_, 0.1f, 10.0f, "%.3f");

		// Optional vertex dots (Processing draws "spheres" at sample points; we do GL points).
		ImGui::Checkbox("Vertex points", &drawVertexDots_);
		ImGui::SliderInt("Point stride", &pointStride_, 1, 6);
		ImGui::SliderFloat("Point size", &pointSize_, 1.0f, 8.0f, "%.1f");

		if (regen)
		{
			generateGeometry();
		}
	}

private:
	std::vector<Vec4f> vertices_;     // local (object) space
	std::vector<float> hyperDists_;  // |v| in 4D, based on local vertices (no translation)

	// Processing defaults: tSteps=3000, vSteps=24
	int tSteps_{3000};
	int vSteps_{24};

	// Path constants
	float GR_{(1.0f + std::sqrt(5.0f)) / 2.0f};
	float E_{2.718281828f};
	float PI_{3.14159265358979323846f};

	// Dynamic multipliers (user-controlled)
	float m1_{1.0f};
	float m2_{1.0f};
	float m3_{1.0f};

	// Growth parameters (Processing defaults)
	float a_{0.05f};
	float b_{0.06f};
	float tubeGrow_{0.06f};
	float tubeScale_{0.6f};
	float maxT_{90.0f};

	// Tangent sampling delta (Processing uses 0.01)
	float deltaTangent_{0.01f};

	// Rendering window (Processing RangeSlider defaults visibleSteps=250)
	bool selectiveDisplay_{true};
	int startRing_{0};
	int visibleRings_{250};

	// Vertex dots (Processing draws every other ring+v; we use a stride knob)
	bool drawVertexDots_{true};
	int pointStride_{2}; // 2 -> roughly match k+=2, v+=2
	float pointSize_{3.0f};

	static float hyperNorm4(const Vec4f& v)
	{
		return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
	}

	static float dot4(const Vec4f& a, const Vec4f& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
	}

	static float clamp01(float x)
	{
		return std::max(0.0f, std::min(1.0f, x));
	}

	static al::Color gradientColor(float t)
	{
		const al::Color inner(1.f, 1.f, 1.f, 1.f);                           // white
		const al::Color outer(1.f, 100.f / 255.f, 0.f, 76.f / 255.f);         // orange-ish + alpha

		al::Color c;
		c.r = inner.r + t * (outer.r - inner.r);
		c.g = inner.g + t * (outer.g - inner.g);
		c.b = inner.b + t * (outer.b - inner.b);
		c.a = inner.a + t * (outer.a - inner.a);
		return c;
	}

	float ringParamT(int ringIndex) const
	{
		if (tSteps_ <= 1)
		{
			return 0.0f;
		}
		const float u = static_cast<float>(ringIndex) / static_cast<float>(tSteps_ - 1);
		return u * maxT_;
	}

	Vec4f getSpiralPoint(float t) const
	{
		const float r = a_ * std::exp(b_ * t);

		const float theta = (GR_ * m1_) * t;
		const float phi = (E_ * m2_) * t;
		const float chi = (PI_ * m3_) * t;

		const float x = r * std::cos(chi);
		const float y = r * std::sin(chi) * std::cos(phi);
		const float z = r * std::sin(chi) * std::sin(phi) * std::cos(theta);
		const float w = r * std::sin(chi) * std::sin(phi) * std::sin(theta);

		return Vec4f(x, y, z, w);
	}

	void generateGeometry()
	{
		vertices_.clear();
		hyperDists_.clear();

		vertices_.reserve(static_cast<size_t>(tSteps_) * static_cast<size_t>(vSteps_));
		hyperDists_.reserve(static_cast<size_t>(tSteps_) * static_cast<size_t>(vSteps_));

		for (int i = 0; i < tSteps_; ++i)
		{
			const float t = (tSteps_ > 1)
				? (static_cast<float>(i) / static_cast<float>(tSteps_ - 1)) * maxT_
				: 0.f;

			const Vec4f center = getSpiralPoint(t);
			const Vec4f nextP = getSpiralPoint(t + deltaTangent_);

			Vec4f tangent = nextP - center;
			const float tMag = hyperNorm4(tangent);
			if (tMag > 1e-7f)
			{
				tangent *= (1.f / tMag);
			}

			// Choose a stable axis to build a basis orthogonal to tangent.
			Vec4f aAxis(1.f, 0.f, 0.f, 0.f);
			const bool useY = (std::fabs(tangent.x) > 0.9f);
			if (useY)
			{
				aAxis = Vec4f(0.f, 1.f, 0.f, 0.f);
			}

			Vec4f bAxis(0.f, 1.f, 0.f, 0.f);
			if (useY)
			{
				bAxis = Vec4f(0.f, 0.f, 1.f, 0.f);
			}

			// n1 = normalize(aAxis - proj(tangent) * aAxis)
			const float dot1 = dot4(aAxis, tangent);
			Vec4f n1 = aAxis - tangent * dot1;
			const float n1Mag = hyperNorm4(n1);
			if (n1Mag > 1e-7f)
			{
				n1 *= (1.f / n1Mag);
			}

			// n2 = normalize( (bAxis - proj(tangent) * bAxis) - proj(n1) )
			const float dotT = dot4(bAxis, tangent);
			Vec4f t2 = bAxis - tangent * dotT;

			const float dotN1 = dot4(t2, n1);
			Vec4f n2 = t2 - n1 * dotN1;
			const float n2Mag = hyperNorm4(n2);
			if (n2Mag > 1e-7f)
			{
				n2 *= (1.f / n2Mag);
			}

			const float rTube = (a_ * std::exp(tubeGrow_ * t)) * tubeScale_;

			for (int j = 0; j < vSteps_; ++j)
			{
				const float vAngle = (vSteps_ > 0)
					? (static_cast<float>(j) / static_cast<float>(vSteps_)) * (2.f * PI_)
					: 0.f;

				const float cv = std::cos(vAngle);
				const float sv = std::sin(vAngle);

				const Vec4f p = center + (n1 * (rTube * cv)) + (n2 * (rTube * sv));
				vertices_.push_back(p);
				hyperDists_.push_back(hyperNorm4(p));
			}
		}
	}

	void drawProjectedWindow(al::Graphics& g, const object4D& viewer, int startRing, int ringCount)
	{
		if (vertices_.empty())
		{
			return;
		}

		const int ringCountClamped = std::max(1, std::min(ringCount, tSteps_));
		const int startRingClamped = std::max(0, std::min(startRing, tSteps_ - 1));

		// We project ringCount + 1 rings so we can draw the longitudinal edges
		// between ring k and ring k+1.
		const int ringsForProj = ringCountClamped + 1;
		const size_t windowVerts = static_cast<size_t>(ringsForProj) * static_cast<size_t>(vSteps_);

		std::vector<al::Vec3f> proj;
		proj.resize(windowVerts);

		std::vector<float> dists;
		dists.resize(windowVerts);

		float minD = std::numeric_limits<float>::max();
		float maxD = std::numeric_limits<float>::lowest();

		for (int k = 0; k <= ringCountClamped; ++k)
		{
			const int r = (startRingClamped + k) % tSteps_;

			for (int v = 0; v < vSteps_; ++v)
			{
				const int localIdx = r * vSteps_ + v;
				const size_t winIdx = static_cast<size_t>(k * vSteps_ + v);

				const Vec4f& vLoc = vertices_[localIdx];
				const Vec4f vWorld = pos + rotationState_.apply(vLoc);

				// 4D -> viewer-local -> 3D.
				const Vec4f vLocal = toViewerLocal(viewer, vWorld);
				proj[winIdx] = projectLocal4Dto3D(vLocal);

				const float d = hyperDists_[localIdx];
				dists[winIdx] = d;

				minD = std::min(minD, d);
				maxD = std::max(maxD, d);
			}
		}

		if (maxD <= minD)
		{
			maxD = minD + 1.0f;
		}

		g.blending(true);
		g.blendTrans();

		// --------------------------
		// Tube edge lines
		// --------------------------
		al::Mesh lines;
		lines.primitive(al::Mesh::LINES);

		for (int k = 0; k < ringCountClamped; ++k)
		{
			for (int v = 0; v < vSteps_; ++v)
			{
				const size_t curr = static_cast<size_t>(k * vSteps_ + v);
				const size_t nextV = static_cast<size_t>(k * vSteps_ + ((v + 1) % vSteps_));
				const size_t nextT = static_cast<size_t>((k + 1) * vSteps_ + v);

				const float d = dists[curr];
				const float t = clamp01((d - minD) / (maxD - minD));
				const al::Color c = gradientColor(t);

				// Circumferential edge (k,v) -> (k,v+1)
				lines.color(c);
				lines.vertex(proj[curr]);
				lines.color(c);
				lines.vertex(proj[nextV]);

				// Longitudinal edge (k,v) -> (k+1,v), but avoid connecting across the
				// "physical end" of the spiral (Processing checks actualRingIndex==tSteps-1).
				const int actualRingIndex = (startRingClamped + k) % tSteps_;
				const bool isPhysicalWrap = (actualRingIndex == tSteps_ - 1);
				if (!isPhysicalWrap)
				{
					lines.color(c);
					lines.vertex(proj[curr]);
					lines.color(c);
					lines.vertex(proj[nextT]);
				}
			}
		}

		// --------------------------
		// Vertex dots (GL_POINTS)
		// --------------------------
		al::Mesh points;
		if (drawVertexDots_ && pointStride_ > 0)
		{
			points.primitive(al::Mesh::POINTS);
			g.pointSize(pointSize_);

			for (int k = 0; k < ringCountClamped; k += std::max(1, pointStride_))
			{
				for (int v = 0; v < vSteps_; v += std::max(1, pointStride_))
				{
					const size_t idx = static_cast<size_t>(k * vSteps_ + v);
					if (idx >= windowVerts)
					{
						continue;
					}

					const float d = dists[idx];
					const float t = clamp01((d - minD) / (maxD - minD));
					const al::Color c = gradientColor(t);

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

		if (drawVertexDots_ && !points.vertices().empty())
		{
			g.meshColor();
			g.draw(points);
		}

		g.blending(false);
	}
};

} // namespace w4d

