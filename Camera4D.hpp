#pragma once

#include "Nav4D.hpp"

using namespace al;

/// Viewer: a Nav4D plus a home pose and world/local frame change.
class Camera4D : public Nav4D
{
private:
	Vec4f mHomePos{0.f, 0.f, 0.f, 0.f};
	Rotation4D mHomeRot = Rotation4D::identity();

public:
	Camera4D() = default;
	Camera4D(const Camera4D &) = default;
	Camera4D &operator=(const Camera4D &) = default;
	virtual ~Camera4D() = default;
	Camera4D(const Vec4f &homePos, const Rotation4D &homeRot)
		: mHomePos(homePos), mHomeRot(homeRot)
	{
		home();
	}

	void setHome()
	{
		mHomePos = pos;
		mHomeRot = rotationState;
	}

	void setHome(const Vec4f &homePos, const Rotation4D &homeRot)
	{
		mHomePos = homePos;
		mHomeRot = homeRot;
		home();
	}

	void home()
	{
		pos = mHomePos;
		rotationState = mHomeRot;
		syncFace();
	}

	Vec4f toLocal(const Vec4f &pWorld) const
	{
		const Rotation4D inverse = rotationState.inverse();
		return inverse.apply(pWorld - pos);
	}

	Vec4f toWorld(const Vec4f &pLocal) const
	{
		return pos + rotationState.apply(pLocal);
	}
};
