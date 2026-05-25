#pragma once

#include "Fmath.hpp"

using namespace al;
/*
 * Abstract base for 4D objects:
 * position, facing direction, and rotation state.
 * position is a 4D vector (x, y, z, w).
 * facing direction is a set of 4 uniform Vec4f,
 *  default as: FaceRight(1, 0, 0, 0), FaceUp(0, 1, 0, 0), FaceForward(0, 0, -1, 0), FaceAna(0, 0, 0, 1).
 *  any of the face directions is always rotationState.apply(defaultFaceDirection) and is updated whenever rotation state changes (via setRotation / applyRotation).
 * Rotation state is a set of 2 uniform quaternions (qL, qR) that represents the rotation in the 4D space.
 */

class object4D
{
public:
    Vec4f pos{0.f, 0.f, 0.f, 0.f};
    Rotation4D rotationState;
    FaceDirection faceDirection;

    object4D() = default;

    virtual ~object4D() = default;

    void setRotation(const Rotation4D &r) { rotationState = r; }
    void applyRotation(const Rotation4D &r) { rotationState.prepend(r); }

    // --- Movement along axes ---

    /** Move along one axis. axis: 0=x, 1=y, 2=z, 3=w. */
    void move(int axis, float amount)
    {
        if (axis >= 0 && axis <= 3)
        {
            pos[axis] += amount;
        }
    }

    /** Move by a delta vector (pos += delta). */
    void moveBy(const Vec4f &delta) { pos += delta; }

    // --- Rotation in the 6 cardinal planes ---

    /** Rotate in the plane spanned by two axes. axis1, axis2 in {0,1,2,3} for
     * x,y,z,w. angleDeg in degrees, any value (wrapped to [-360, 360] then
     * converted to radians). Rotation is accumulated via applyRotation (local frame). */
    void rotatePlane(int axis1, int axis2, float angleDeg)
    {
        float a = std::fmod(angleDeg, 360.f);
        if (a > 180.f)
        {
            a -= 360.f;
        }
        else if (a < -180.f)
        {
            a += 360.f;
        }
        const float rad = a * (3.14159265358979f / 180.f);
        applyRotation(Rotation4D::fromPlaneAngle(axis1, axis2, rad));
    }

    /** Override in subclasses for custom behaviour. */
    virtual void onRotationChanged() {}

    // Don't call every frame
    void syncFaceFromRotation()
    {
        faceDirection.updateFaceDirection(rotationState);
        Vec4f face = rotationState.apply(Vec4f(1.f, 0.f, 0.f, 0.f));
        face.normalize();
    }
};
