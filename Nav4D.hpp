#pragma once

#include "Object4D.hpp"

using namespace al;

/// 4D viewer navigation: home pose + world/local transforms.
/// Motion and spin integration live on Object4D (moveLocal/moveGlobal, rotate*PlaneSpeed, step).
class Nav4D : public Object4D
{
private:
	Vec4f mHomePos{0.f, 0.f, 0.f, 0.f};
	Rotation4D mHomeRot = Rotation4D::identity();

public:
	Nav4D() = default;
    Nav4D(const Nav4D &) = default;
    Nav4D &operator=(const Nav4D &) = default;
    virtual ~Nav4D() = default;

    Nav4D(const Vec4f &homePos, const Rotation4D &homeRot)
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

	// -------- World <-> viewer-local transforms --------

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