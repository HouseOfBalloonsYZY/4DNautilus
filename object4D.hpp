#pragma once

#include "Fmath.hpp"

using namespace al;
/** Abstract base for 4D objects: position, facing direction, and rotation state.
 *  face is always rotationState.apply(1.f, 0.f, 0.f, 0.f) and is updated whenever
 *  rotation state changes (via setRotation / prependRotation). */
class object4D
{
public:
    Vec4f pos{0.f, 0.f, 0.f, 0.f};
    Vec4f face{1.f, 0.f, 0.f, 0.f};

    object4D() = default;

    virtual ~object4D() = default;

    /** Current rotation; read-only. Use setRotation / prependRotation to change. */
    const Rotation4D &getRotation() const
    {
        return rotationState_;
    }

    /** Set rotation and sync face to rotationState.apply(defaultForward). */
    void setRotation(const Rotation4D &r)
    {
        rotationState_ = r;
        syncFaceFromRotation();
    }

    /** Prepend a rotation (apply after current) and sync face. */
    void prependRotation(const Rotation4D &r)
    {
        rotationState_.prepend(r);
        syncFaceFromRotation();
    }

    /** Compose rotation on the right (this * r) and sync face. */
    void appendRotation(const Rotation4D &r)
    {
        rotationState_ = rotationState_ * r;
        syncFaceFromRotation();
    }

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
    void moveBy(const Vec4f &delta)
    {
        pos += delta;
    }

    // --- Rotation in the 6 cardinal planes ---

    /** Rotate in the plane spanned by two axes. axis1, axis2 in {0,1,2,3} for x,y,z,w.
     *  angleDeg in degrees, any value (wrapped to [-360, 360] then converted to radians).
     *  Rotation is prepended (applied in local frame). */
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
        prependRotation(Rotation4D::fromPlaneAngle(axis1, axis2, rad));
    }

protected:
    /** Override in subclasses for custom behaviour. */
    virtual void onRotationChanged() {}

    /** Recompute face from rotationState. Call after any direct mutation of rotationState_. */
    void syncFaceFromRotation()
    {
        face = rotationState_.apply(Vec4f(1.f, 0.f, 0.f, 0.f));
        face.normalize();
        onRotationChanged();
    }

    Rotation4D rotationState_;
};
