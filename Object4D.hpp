#pragma once

#include "FMath.hpp"
#include <array>
#include <cmath>

using namespace al;

/*
 * 4D object: world pose (pos, rotationState) plus viewer-local face basis.
 * Motion / spin API:
 *   - *By*     : immediate displacement / rotation (no smoothing)
 *   - *Speed*  : target velocity / plane rates, integrated in step(dt) with smoothing + clamps
 */

std::array<std::pair<int, int>, 6> kPlane6 = {{{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}}};

class Object4D
{
protected:
    float mSmooth = 0.5f;
    float mMaxMoveSpeed = 3.0f;
    float mMaxRotateSpeed = 6.283185307f; // 2*pi

    // Target velocities (units/sec, rad/sec).
    Vec4f mMoveSpeedGlobal{0.f, 0.f, 0.f, 0.f};
    Vec4f mMoveSpeedLocal{0.f, 0.f, 0.f, 0.f};
    std::array<float, 6> mRotateSpeedGlobal{};
    std::array<float, 6> mRotateSpeedLocal{};

    // Smoothed displacement / angle increment for the current step (Nav-style mMove1 / mSpin1).
    Vec4f mMoveStepGlobal{0.f, 0.f, 0.f, 0.f};
    Vec4f mMoveStepLocal{0.f, 0.f, 0.f, 0.f};
    std::array<float, 6> mRotateStepGlobal{};
    std::array<float, 6> mRotateStepLocal{};

    // Pending one-shot *By* buffers (consumed when applied).
    Vec4f mMoveByGlobal{0.f, 0.f, 0.f, 0.f};
    Vec4f mMoveByLocal{0.f, 0.f, 0.f, 0.f};
    std::array<float, 6> mRotateByGlobal{};
    std::array<float, 6> mRotateByLocal{};

    void syncFace() { faceDirection.updateFaceDirection(rotationState); }

    void setRotationState(const Rotation4D &r) { rotationState = r; }
    void appendRotationState(const Rotation4D &delta) { rotationState.append(delta); }
    void composeRotationLocal(const Rotation4D &L) { rotationState.composeRight(L); }

    // ------------------------------------------------------------
    // hidden interfaces ------------------------------------------
    // ------------------------------------------------------------
    // -- by --
    void moveByGlobal()
    {
        pos += mMoveByGlobal;

        mMoveByGlobal.set(0);
        onPositionChanged();
    }

    void moveByLocal()
    {
        for (int i = 0; i < 4; ++i)
        {
            pos += faceDirection.face[i] * mMoveByLocal[i];
        }

        mMoveByLocal.set(0);
        onPositionChanged();
    }

    // these two r old
    void rotateByGlobal(int axis1, int axis2, float angleDeg)
    {
        float rad = degreesToRad(angleDeg);

        appendRotationState(Rotation4D::fromGlobalPlane(axis1, axis2, rad));

        onRotationChanged();
    }

    void rotateByLocal(int axis1, int axis2, float angleDeg)
    {
        float rad = degreesToRad(angleDeg);
        // Body-local: R ← R ∘ L (cardinal L, no face-dependent M).
        composeRotationLocal(Rotation4D::fromBodyPlane(axis1, axis2, rad));
        syncFace();
        onRotationChanged();
    }

    void rotateByGlobal()
    {
        for (int i = 0; i < 6; ++i)
        {
            float rad = mRotateByGlobal[i];
            if (rad == 0.f)
            {
                continue;
            }
            appendRotationState(Rotation4D::fromGlobalPlane(kPlane6[i].first, kPlane6[i].second, rad));
        }

        mRotateByGlobal.fill(0.f);

        onRotationChanged();
    }

    void rotateByLocal()
    {
        for (int i = 0; i < 6; ++i)
        {
            float rad = mRotateByLocal[i];
            if (rad == 0.f)
            {
                continue;
            }
            composeRotationLocal(Rotation4D::fromBodyPlane(kPlane6[i].first, kPlane6[i].second, rad));
        }

        mRotateByLocal.fill(0.f);
        syncFace();
        onRotationChanged();
    }

    // -- speed -- (Nav: mMove1.lerp(mMove0*dt + nudge, amt); then apply)
    void moveStepGlobal(float dt)
    {
        const float amt = 1.f - mSmooth;
        mMoveStepGlobal.lerp(mMoveSpeedGlobal * dt, amt);
        pos += mMoveStepGlobal;
        onPositionChanged();
    }

    void moveStepLocal(float dt)
    {
        const float amt = 1.f - mSmooth;
        mMoveStepLocal.lerp(mMoveSpeedLocal * dt, amt);
        for (int i = 0; i < 4; ++i)
        {
            pos += faceDirection.face[i] * mMoveStepLocal[i];
        }
        onPositionChanged();
    }

    void rotateStepGlobal(float dt)
    {
        const float amt = 1.f - mSmooth;
        for (int i = 0; i < 6; ++i)
        {
            // lerp
            mRotateStepGlobal[i] += (mRotateSpeedGlobal[i] * dt - mRotateStepGlobal[i]) * amt;
        }

        const float eps = 1e-7f; // super small non-zero threshold for negligible angle

        for (int i = 0; i < 6; ++i)
        {
            if (mRotateStepGlobal[i] > eps || mRotateStepGlobal[i] < -eps)
            {
                appendRotationState(Rotation4D::fromGlobalPlane(kPlane6[i].first, kPlane6[i].second, mRotateStepGlobal[i]));
            }
        }

        onRotationChanged();
    }

    void rotateStepLocal(float dt)
    {
        const float amt = 1.f - mSmooth;
        for (int i = 0; i < 6; ++i)
        {
            // lerp
            mRotateStepLocal[i] += (mRotateSpeedLocal[i] * dt - mRotateStepLocal[i]) * amt;
        }

        const float eps = 1e-7f; // super small non-zero threshold for negligible angle

        for (int i = 0; i < 6; ++i)
        {
            if (mRotateStepLocal[i] > eps || mRotateStepLocal[i] < -eps)
            {
                // Body-local: R ← R ∘ L_cardinal (no fromLocalPlane / face M).
                composeRotationLocal(Rotation4D::fromBodyPlane(
                    kPlane6[i].first, kPlane6[i].second, mRotateStepLocal[i]));
            }
        }

        syncFace();
        onRotationChanged();
    }

    // ------------------------------------------------------------
    // ------------------------------------------------------------
    // ------------------------------------------------------------

public:
    Vec4f pos{0.f, 0.f, 0.f, 0.f};
    Rotation4D rotationState;
    FaceDirection faceDirection;

    Object4D() = default;
    virtual ~Object4D() = default;

    virtual void onPositionChanged() {}
    virtual void onRotationChanged() {}

    // ------------------------------------------------------------
    // public APIs ------------------------------------------------
    // ------------------------------------------------------------
    // -- by --
    void setMoveByGlobal(const Vec4f &delta)
    {
        mMoveByGlobal = delta;
        moveByGlobal();
    }

    void addMoveByGlobal(int i, float delta)
    {
        if (i < 0 || i > 3)
        {
            return;
        }
        mMoveByGlobal[i] += delta;
        moveByGlobal();
    }

    void setMoveByLocal(const Vec4f &delta)
    {
        mMoveByLocal = delta;
        moveByLocal();
    }

    void addMoveByLocal(int i, float delta)
    {
        if (i < 0 || i > 3)
        {
            return;
        }
        mMoveByLocal[i] += delta;
        moveByLocal();
    }

    void setRotationByGlobal(const std::array<float, 6> &delta)
    {
        mRotateByGlobal = delta;
        rotateByGlobal();
    }

    void setRotationByLocal(const std::array<float, 6> &delta)
    {
        mRotateByLocal = delta;
        rotateByLocal();
    }

    // -- speed --
    void setMoveSpeedGlobal(const Vec4f &speed)
    {
        mMoveSpeedGlobal = clampVec4(speed, mMaxMoveSpeed);
    }

    void addMoveSpeedGlobal(const Vec4f &delta)
    {
        mMoveSpeedGlobal += delta;
        mMoveSpeedGlobal = clampVec4(mMoveSpeedGlobal, mMaxMoveSpeed);
    }

    void addMoveSpeedGlobal(int i, float delta)
    {
        if (i < 0 || i > 3)
        {
            return;
        }
        mMoveSpeedGlobal[i] += delta;
        mMoveSpeedGlobal = clampVec4(mMoveSpeedGlobal, mMaxMoveSpeed);
    }

    void setMoveSpeedLocal(const Vec4f &speed)
    {
        mMoveSpeedLocal = clampVec4(speed, mMaxMoveSpeed);
    }

    void addMoveSpeedLocal(const Vec4f &delta)
    {
        mMoveSpeedLocal += delta;
        mMoveSpeedLocal = clampVec4(mMoveSpeedLocal, mMaxMoveSpeed);
    }

    void addMoveSpeedLocal(int i, float delta)
    {
        if (i < 0 || i > 3)
        {
            return;
        }
        mMoveSpeedLocal[i] += delta;
        mMoveSpeedLocal = clampVec4(mMoveSpeedLocal, mMaxMoveSpeed);
    }

    void setRotateSpeedGlobal(const std::array<float, 6> &speed)
    {
        mRotateSpeedGlobal = clampArray6(speed, mMaxRotateSpeed);
    }

    void addRotateSpeedGlobal(const std::array<float, 6> &delta)
    {
        addArray6(mRotateSpeedGlobal, delta);
        mRotateSpeedGlobal = clampArray6(mRotateSpeedGlobal, mMaxRotateSpeed);
    }

    void addRotateSpeedGlobal(int i, float delta)
    {
        if (i < 0 || i > 5)
        {
            return;
        }
        mRotateSpeedGlobal[i] += delta;
        mRotateSpeedGlobal = clampArray6(mRotateSpeedGlobal, mMaxRotateSpeed);
    }

    void setRotateSpeedLocal(const std::array<float, 6> &speed)
    {
        mRotateSpeedLocal = clampArray6(speed, mMaxRotateSpeed);
    }

    void addRotateSpeedLocal(const std::array<float, 6> &delta)
    {
        addArray6(mRotateSpeedLocal, delta);
        mRotateSpeedLocal = clampArray6(mRotateSpeedLocal, mMaxRotateSpeed);
    }

    void addRotateSpeedLocal(int i, float delta)
    {
        if (i < 0 || i > 5)
        {
            return;
        }
        mRotateSpeedLocal[i] += delta;
        mRotateSpeedLocal = clampArray6(mRotateSpeedLocal, mMaxRotateSpeed);
    }

    // ------------------------------------------------------------
    // ------------------------------------------------------------
    // ------------------------------------------------------------



    // TODO: add nudge feature

    

    /// Integrate *Speed* over dt (one-pole low-pass, then apply). Call once per frame.
    void step(double dt)
    {
        const float fdt = static_cast<float>(dt);
        if (fdt == 0.f)
        {
            return;
        }

        // Nav applies rotation before translation.
        rotateStepGlobal(fdt);
        rotateStepLocal(fdt);
        moveStepGlobal(fdt);
        moveStepLocal(fdt);

        onPositionChanged();
        onRotationChanged();
    }

    // stop
    void halt()
    {
        mMoveSpeedGlobal.set(0);
        mMoveSpeedLocal.set(0);
        mMoveStepGlobal.set(0);
        mMoveStepLocal.set(0);
        mRotateSpeedGlobal.fill(0.f);
        mRotateSpeedLocal.fill(0.f);
        mRotateStepGlobal.fill(0.f);
        mRotateStepLocal.fill(0.f);

        mMoveByGlobal.set(0);
        mMoveByLocal.set(0);
        mRotateByGlobal.fill(0.f);
        mRotateByLocal.fill(0.f);
    }
};
