#pragma once

#include "Nav4D.hpp"
#include "Object4D.hpp"

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

using namespace al;

// ----------------------------------------------------------------------------
// Mesh4D — 4D geometry container for localization → reduction (not al::Graphics).
//
// Pipeline (intended):
//   generateGeometry()     → vertices + elements (object-local)
//   syncVertexBuffers()    → resize world / Nav4D buffers to match vertices
//   updateWorld()          → verticesWorld from pose + vertices
//   updateNav4D(nav)       → verticesNav4D from Nav4D::toLocal (call each frame)
//
// Primitives name intrinsic simplices in R⁴ (indexed into vertices), analogous
// in role to al::Mesh::Primitive — but NOT OpenGL draw modes (see comment below).
// ----------------------------------------------------------------------------

using Index4D = int;

/// 1-simplex: segment between two vertices.
using Edge4D = std::pair<Index4D, Index4D>;

/// 2-simplex: triangle (2D flat facet or filled 2-cell in 4D).
struct Triangle4D
{
	std::array<Index4D, 3> i{};
};

/// 3-simplex: tetrahedron (3D flat facet or filled 3-cell in 4D).
struct Tetrahedron4D
{
	std::array<Index4D, 4> i{};
};

/// 4-simplex: pentachoron / 5-cell (true 4D volumetric cell; hypervolume slicing).
struct Pentachoron4D
{
	std::array<Index4D, 5> i{};
};

/// Intrinsic simplex types stored in Mesh4D (not GL_POINTS / GL_TRIANGLES).
enum class Primitive4D : unsigned int
{
	Edge = 0,
	Triangle = 1,
	Tetrahedron = 2,
	Pentachoron = 3
};

/// Indexed element soup — reducers consume the buckets they need.
struct Elements4D
{
	std::vector<Edge4D> edges;
	std::vector<Triangle4D> triangles;
	std::vector<Tetrahedron4D> tetrahedra;
	std::vector<Pentachoron4D> pentachora;

	void clear()
	{
		edges.clear();
		triangles.clear();
		tetrahedra.clear();
		pentachora.clear();
	}

	bool empty() const
	{
		return edges.empty() && triangles.empty() && tetrahedra.empty() && pentachora.empty();
	}
};

class Mesh4D
{
public:
	Object4D pose;

	/// Primary element kind for generators that emit one homogeneous type (optional hint).
	Primitive4D primitive{Primitive4D::Edge};

	/// Object-local generated vertices (authorship space).
	std::vector<Vec4f> vertices;

	/// World 4D positions (pose applied); parallel to vertices.
	std::vector<Vec4f> verticesWorld;

	/// Nav4D viewer-local positions; parallel to vertices.
	std::vector<Vec4f> verticesNav4D;

	/// Index connectivity into vertices / verticesNav4D (same indexing).
	Elements4D elements;

	Mesh4D() = default;
	explicit Mesh4D(Primitive4D p)
		: primitive(p)
	{
	}

	Mesh4D(const Mesh4D &) = default;
	Mesh4D &operator=(const Mesh4D &) = default;
	virtual ~Mesh4D() = default;

	// --- Accessors ---

	Primitive4D primitiveType() const { return primitive; }
	void setPrimitiveType(Primitive4D p) { primitive = p; }

	const std::vector<Vec4f> &verticesLocal() const { return vertices; }
	const std::vector<Vec4f> &verticesInWorld() const { return verticesWorld; }
	const std::vector<Vec4f> &verticesInNav4D() const { return verticesNav4D; }
	const Elements4D &elementsData() const { return elements; }

	size_t vertexCount() const { return vertices.size(); }

	// --- Geometry generation (subclasses override) ---

	/// Fill vertices and elements in object-local space. Does not touch world / Nav4D buffers.
	virtual void generateGeometry()
	{
	}

	/// Regenerate local geometry and refresh derived vertex buffers from current pose.
	void rebuildFromPose()
	{
		generateGeometry();
		syncVertexBuffers();
		updateWorld();
	}

	// --- Buffer sync (World 1 → World 2 positions) ---

	/// Ensure world / Nav4D buffers match vertices.size() (safe before update loops).
	void syncVertexBuffers()
	{
		const size_t n = vertices.size();
		verticesWorld.resize(n);
		verticesNav4D.resize(n);
	}

	/// World 4D from object-local + pose. Call when pose.pos or pose.rotationState changes.
	void updateWorld()
	{
		syncVertexBuffers();

		for (size_t i = 0; i < vertices.size(); ++i)
		{
			verticesWorld[i] = pose.pos + pose.rotationState.apply(vertices[i]);
		}
	}

	/// Viewer-local 4D from world positions. Call each frame from FApp after Nav4D moves.
	void updateNav4D(const Nav4D &nav)
	{
		if (verticesWorld.size() != vertices.size())
		{
			updateWorld();
		}
		else
		{
			syncVertexBuffers();
		}

		for (size_t i = 0; i < verticesWorld.size(); ++i)
		{
			verticesNav4D[i] = nav.toLocal(verticesWorld[i]);
		}
	}

	/// Full localization pass: world from pose, then Nav4D capture.
	void updateNav4DFromPose(const Nav4D &nav)
	{
		updateWorld();
		updateNav4D(nav);
	}

	void clear()
	{
		vertices.clear();
		verticesWorld.clear();
		verticesNav4D.clear();
		elements.clear();
	}

protected:
	void addEdge(Index4D a, Index4D b)
	{
		elements.edges.emplace_back(a, b);
	}

	void addTriangle(Index4D a, Index4D b, Index4D c)
	{
		elements.triangles.push_back(Triangle4D{{a, b, c}});
	}

	void addTetrahedron(Index4D a, Index4D b, Index4D c, Index4D d)
	{
		elements.tetrahedra.push_back(Tetrahedron4D{{a, b, c, d}});
	}

	void addPentachoron(Index4D a, Index4D b, Index4D c, Index4D d, Index4D e)
	{
		elements.pentachora.push_back(Pentachoron4D{{a, b, c, d, e}});
	}
};
