#pragma once

#include "FMath.hpp"

using namespace al;

// Rigid 4D placement: position, rotation, and the body face basis.
// No velocities. Mesh pose and generated objects use this.
class Pose4D
{
protected:
	void syncFace() { faceDirection.updateFaceDirection(rotationState); }

	void setRotationState(const Rotation4D &r) { rotationState = r; }
	void appendRotationState(const Rotation4D &delta) { rotationState.append(delta); }
	void prependRotationState(const Rotation4D &delta) { rotationState.prepend(delta); }

public:
	Vec4f pos{0.f, 0.f, 0.f, 0.f};
	Rotation4D rotationState;
	FaceDirection faceDirection;

	Pose4D() = default;
	virtual ~Pose4D() = default;

	virtual void onPositionChanged() {}
	virtual void onRotationChanged() {}
};
