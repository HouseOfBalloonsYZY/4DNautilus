#pragma once

#include "Camera4D.hpp"
#include "Mesh4D.hpp"

#include <vector>

using namespace al;

// Every drawable 4D object in the world. Reducers read this list, not generators.
class Scene4D
{
public:
	void clear()
	{
		meshes_.clear();
	}

	void add(Mesh4D &mesh)
	{
		meshes_.push_back(&mesh);
	}

	const std::vector<Mesh4D *> &meshes() const { return meshes_; }

	/// Identity field, then pose, then camera localization.
	void prepare(const Camera4D &camera)
	{
		for (Mesh4D *mesh : meshes_)
		{
			mesh->copyGeneratedToDeformed();
			mesh->updateNav4DFromPose(camera);
		}
	}

private:
	std::vector<Mesh4D *> meshes_;
};
