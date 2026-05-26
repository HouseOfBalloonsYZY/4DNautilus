#pragma once

#include <vector>

#include "Fmath.hpp"
#include "Object4D.hpp"

#include <cmath>

using namespace al;

// ---------------------------------------------------------------------------
// Simple 4D Spherinder manifold (sphere extruded along w).
// Local vertices live in object space; world position and rotation come from
// Object4D.
// ---------------------------------------------------------------------------

class Spherinder4D : public Object4D
{
public:
	Spherinder4D()
	{
		generateGeometry();
	}

	const std::vector<Vec4f> &verticesLocal() const { return vertices_; }
	const std::vector<std::pair<int, int>> &edgesLocal() const { return edges_; }

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
