#pragma once

#include "Mesh4D.hpp"
#include "al/io/al_Imgui.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace al;

// 4D nautilus as a soft-cell chamber along the logarithmic spiral.
// Geometry is written only into Mesh4D::verticesGenerated.
// Cross-section: 3D soft cell (smooth wall, two pinched tips), after
// Domokos et al., https://www.nature.com/articles/d41586-024-03099-6
class Nautilus4D
{
	float GR_{(1.f + std::sqrt(5.f)) / 2.f};
	float E_{2.718281828f};
	float PI_{3.14159265358979323846f};

	float m1_{1.f};
	float m2_{1.f};
	float m3_{1.f};

	float a_{0.05f};
	float b_{0.06f};
	float tubeGrow_{0.06f};
	float tubeScale_{0.6f};
	float maxT_{90.f};
	float deltaTangent_{0.01f};

	bool selectiveDisplay_{true};
	int startRing_{0};
	int visibleRings_{250};
	int volumeRingCount_{0};

	static float hyperNorm4(const Vec4f &v)
	{
		return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
	}

	static float dot4(const Vec4f &a, const Vec4f &b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
	}

	Vec4f getSpiralPoint(float t) const
	{
		const float r = a_ * std::exp(b_ * t);
		const float theta = (GR_ * m1_) * t;
		const float phi = (E_ * m2_) * t;
		const float chi = (PI_ * m3_) * t;
		return Vec4f(
			r * std::cos(chi),
			r * std::sin(chi) * std::cos(phi),
			r * std::sin(chi) * std::sin(phi) * std::cos(theta),
			r * std::sin(chi) * std::sin(phi) * std::sin(theta));
	}

public:
    // Soft-cell chamber along the spiral (Domokos et al., the nautilus shape in
    // https://www.nature.com/articles/d41586-024-03099-6 ). Cross-section in the
    // normal 3-space is smooth with two pinched tips. Many short steps follow the
    // curve. Topologically each chamber is still a 4-ball. Written only into Mesh4D.
    // The outer aperture has no septum.
	void publishVolumeMesh(Mesh4D &mesh)
    {
        mesh.clear();
        mesh.setPrimitiveType(Primitive4D::Pentachoron);
		mesh.verticesGenerated.reserve(static_cast<size_t>(161 * 65));

		const int chamberSteps[] = {16, 24, 32, 40, 48};
		int totalSegments = 0;
		std::vector<int> chamberStart;
		for (int steps : chamberSteps)
		{
			chamberStart.push_back(totalSegments);
			totalSegments += steps;
		}
		const int kRings = totalSegments + 1;
		volumeRingCount_ = kRings;

		const int windowStart = selectiveDisplay_
									? std::max(0, std::min(startRing_, kRings - 1))
									: 0;
		const int windowCount = selectiveDisplay_
									? std::max(1, std::min(visibleRings_, kRings - windowStart))
									: kRings;
		const int windowEnd = windowStart + windowCount;
		auto ringVisible = [&](int ring)
		{
			return ring >= windowStart && ring < windowEnd;
		};

        auto frameAt = [&](float t, Vec4f &center, Vec4f &n1, Vec4f &n2, Vec4f &n3, float &radius)
        {
            center = getSpiralPoint(t);
            Vec4f tangent = getSpiralPoint(t + deltaTangent_) - center;
            const float tMag = hyperNorm4(tangent);
            if (tMag > 1e-7f)
            {
                tangent *= (1.f / tMag);
            }

            Vec4f aAxis(1.f, 0.f, 0.f, 0.f);
            Vec4f bAxis(0.f, 1.f, 0.f, 0.f);
            if (std::fabs(tangent.x) > 0.9f)
            {
                aAxis = Vec4f(0.f, 1.f, 0.f, 0.f);
                bAxis = Vec4f(0.f, 0.f, 1.f, 0.f);
            }

            n1 = aAxis - tangent * dot4(aAxis, tangent);
            const float n1Mag = hyperNorm4(n1);
            if (n1Mag > 1e-7f)
            {
                n1 *= (1.f / n1Mag);
            }

            Vec4f t2 = bAxis - tangent * dot4(bAxis, tangent);
            n2 = t2 - n1 * dot4(t2, n1);
            const float n2Mag = hyperNorm4(n2);
            if (n2Mag > 1e-7f)
            {
                n2 *= (1.f / n2Mag);
            }

            n3 = cross4(tangent, n1, n2);
            const float n3Mag = hyperNorm4(n3);
            if (n3Mag > 1e-7f)
            {
                n3 *= (1.f / n3Mag);
            }

            radius = (a_ * std::exp(tubeGrow_ * t)) * tubeScale_;
        };

		Vec4f prevN1;
		Vec4f prevN2;
		Vec4f prevN3;
		bool haveFrame = false;

		for (int ring = 0; ring < kRings; ++ring)
		{
			const float t = (totalSegments > 0)
								? (static_cast<float>(ring) / static_cast<float>(totalSegments)) * maxT_
								: 0.f;
			Vec4f center;
			Vec4f n1;
			Vec4f n2;
			Vec4f n3;
			float radius = 0.f;
			frameAt(t, center, n1, n2, n3, radius);
			if (haveFrame)
			{
				Vec4f tangent = getSpiralPoint(t + deltaTangent_) - center;
				const float tMag = hyperNorm4(tangent);
				if (tMag > 1e-7f)
				{
					tangent *= (1.f / tMag);
				}
				n1 = prevN1 - tangent * dot4(prevN1, tangent);
				const float n1Mag = hyperNorm4(n1);
				if (n1Mag > 1e-7f)
				{
					n1 *= (1.f / n1Mag);
				}
				n2 = prevN2 - tangent * dot4(prevN2, tangent) - n1 * dot4(prevN2, n1);
				const float n2Mag = hyperNorm4(n2);
				if (n2Mag > 1e-7f)
				{
					n2 *= (1.f / n2Mag);
				}
				n3 = cross4(tangent, n1, n2);
				const float n3Mag = hyperNorm4(n3);
				if (n3Mag > 1e-7f)
				{
					n3 *= (1.f / n3Mag);
				}
			}
			prevN1 = n1;
			prevN2 = n2;
			prevN3 = n3;
			haveFrame = true;

            mesh.verticesGenerated.push_back(center);
            const int A = 16;
            const int H = 4;
            for (int h = 0; h < H; ++h)
            {
                const float phi = 3.14159265f * static_cast<float>(h) / static_cast<float>(H - 1);
                const float sp = std::sin(phi);
                const float cp = std::cos(phi);
                for (int a = 0; a < A; ++a)
                {
                    const float theta = (2.f * PI_) * static_cast<float>(a) / static_cast<float>(A);
                    // Two cusps, at theta = 0 and pi. |sin|^0.4 is the soft-cell pinch:
                    // smooth belly, pointed tips. Thickness in n3 uses the same pinch,
                    // so the tips are sharp edges rather than cube corners.
                    const float pinch = std::pow(std::fabs(std::sin(theta)), 0.4f);
                    const float radial = radius * (0.08f + 0.92f * pinch);
                    const float height = radius * 0.7f * pinch;
                    mesh.verticesGenerated.push_back(
                        center
                        + n1 * (radial * std::cos(theta) * sp)
                        + n2 * (radial * std::sin(theta) * sp)
                        + n3 * (height * cp));
                }
            }
        }

        constexpr int A = 16;
        constexpr int H = 4;
        constexpr int kStride = 1 + A * H;
        auto centerIndex = [&](int ring) { return ring * kStride; };
        auto surfIndex = [&](int ring, int a, int h)
        {
            return ring * kStride + 1 + h * A + a;
        };

        for (int ring = 0; ring < kRings; ++ring)
        {
            if (!ringVisible(ring))
            {
                continue;
            }
            for (int h = 0; h < H; ++h)
            {
                for (int a = 0; a < A; ++a)
                {
                    const int nextA = (a + 1) % A;
                    mesh.elements.edges.emplace_back(surfIndex(ring, a, h), surfIndex(ring, nextA, h));
                    if (h + 1 < H)
                    {
                        mesh.elements.edges.emplace_back(surfIndex(ring, a, h), surfIndex(ring, a, h + 1));
                    }
                    mesh.elements.edges.emplace_back(centerIndex(ring), surfIndex(ring, a, h));
                }
            }
            if (ring == kRings - 1)
            {
                continue;
            }
        }

        for (int ring = 0; ring < kRings - 1; ++ring)
        {
            if (!ringVisible(ring) || !ringVisible(ring + 1))
            {
                continue;
            }
            for (int h = 0; h < H; ++h)
            {
                for (int a = 0; a < A; ++a)
                {
                    mesh.elements.edges.emplace_back(surfIndex(ring, a, h), surfIndex(ring + 1, a, h));
                }
            }
            for (int h = 0; h + 1 < H; ++h)
            {
                for (int a = 0; a < A; ++a)
                {
                    const int nextA = (a + 1) % A;
                    const int v00 = surfIndex(ring, a, h);
                    const int v10 = surfIndex(ring, nextA, h);
                    const int v11 = surfIndex(ring + 1, nextA, h);
                    const int v01 = surfIndex(ring + 1, a, h);
                    mesh.elements.triangles.push_back(Triangle4D{{v00, v10, v11}});
                    mesh.elements.triangles.push_back(Triangle4D{{v00, v11, v01}});
                    const int w00 = surfIndex(ring, a, h + 1);
                    mesh.elements.pentachora.push_back(Pentachoron4D{{
                        centerIndex(ring), v00, v10, v11, w00}});
                }
            }
        }

        for (int start : chamberStart)
        {
            if (start >= kRings - 1 || !ringVisible(start))
            {
                continue;
            }
            const int mid = H / 2;
            for (int a = 0; a < A; ++a)
            {
                const int nextA = (a + 1) % A;
                mesh.elements.tetrahedra.push_back(Tetrahedron4D{{
                    centerIndex(start),
                    surfIndex(start, a, mid),
                    surfIndex(start, nextA, mid),
                    surfIndex(start, a, std::min(H - 1, mid + 1))}});
            }
        }

        mesh.copyGeneratedToDeformed();
    }


	int projectionStartRing() const
	{
		const int n = std::max(1, volumeRingCount_);
		return std::max(0, std::min(startRing_, n - 1));
	}

	int visibleRingCountFor(int ringMax) const
	{
		if (!selectiveDisplay_)
		{
			return ringMax;
		}
		const int start = std::max(0, std::min(startRing_, std::max(0, ringMax - 1)));
		return std::max(1, std::min(visibleRings_, ringMax - start));
	}

public:
	void drawImGuiControls()
	{
		ImGui::Separator();
		ImGui::Text("4D Nautilus (soft-cell chambers)");
		const int ringMax = std::max(1, volumeRingCount_);
		ImGui::Text("Total rings: %d", volumeRingCount_);
		ImGui::Checkbox("Ring window only", &selectiveDisplay_);
		ImGui::SameLine();
		if (ImGui::Button("Show all rings"))
		{
			selectiveDisplay_ = false;
		}
		if (selectiveDisplay_)
		{
			ImGui::SliderInt("Start ring", &startRing_, 0, std::max(0, ringMax - 1));
			visibleRings_ = std::max(1, std::min(visibleRings_, ringMax));
			ImGui::SliderInt("Visible rings", &visibleRings_, 1, ringMax);
			ImGui::Text(
				"Drawing rings %d .. %d (%d rings)",
				projectionStartRing(),
				std::min(ringMax, projectionStartRing() + visibleRingCountFor(ringMax)) - 1,
				visibleRingCountFor(ringMax));
		}
		else
		{
			ImGui::TextUnformatted("Drawing all rings");
		}
		ImGui::SliderFloat("m1 (GR)", &m1_, 0.1f, 10.f, "%.3f");
		ImGui::SliderFloat("m2 (E)", &m2_, 0.1f, 10.f, "%.3f");
		ImGui::SliderFloat("m3 (PI)", &m3_, 0.1f, 10.f, "%.3f");
	}

private:
    static float det3(
        float a, float b, float c,
        float d, float e, float f,
        float g, float h, float i)
    {
        return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    }

    static Vec4f cross4(const Vec4f &u, const Vec4f &v, const Vec4f &w)
    {
        return Vec4f(
            det3(u.y, u.z, u.w, v.y, v.z, v.w, w.y, w.z, w.w),
            -det3(u.x, u.z, u.w, v.x, v.z, v.w, w.x, w.z, w.w),
            det3(u.x, u.y, u.w, v.x, v.y, v.w, w.x, w.y, w.w),
            -det3(u.x, u.y, u.z, v.x, v.y, v.z, w.x, w.y, w.z));
    }

};
