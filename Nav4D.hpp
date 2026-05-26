#pragma once

#include "Fmath.hpp"
#include "Object4D.hpp"

#include <array>

using namespace al;

/// 4D viewer navigation controller (4D camera).
/// - Stores a smoothed translational velocity in local coordinates.
/// - Stores smoothed rotational plane rates in local coordinates.
/// - step(dt) integrates to update:
///   - Object4D::pos
///   - Object4D::rotationState (6-plane rotations in local frame)
/// - Provides world <-> viewer-local transforms for 4D reducers.
class Nav4D : public Object4D
{
private:
	// Smoothing coefficient in [0,1).
	float mSmooth = 0.5f;

	// Translation in local coordinates (x,y,z,w) and displacement for this step.
	// mMove0: velocity (units/sec). mMove1: displacement for current step.
	Vec4f mMove0{0.f, 0.f, 0.f, 0.f};
	Vec4f mMove1{0.f, 0.f, 0.f, 0.f};
	Vec4f mNudge{0.f, 0.f, 0.f, 0.f};

	// Rotation in local coordinates:
	// plane rates correspond to Rotation4D::fromRates inputs (rXY,rYZ,rZX,rXW,rYW,rZW).
	std::array<float, 6> mSpin0{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; // rad/sec
	std::array<float, 6> mSpin1{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; // rad increment for this step
	std::array<float, 6> mTurn{0.f, 0.f, 0.f, 0.f, 0.f, 0.f};  // one-shot increment

	// Home pose.
	Vec4f mHomePos{0.f, 0.f, 0.f, 0.f};
	Rotation4D mHomeRot = Rotation4D::identity();
public:
	Nav4D()
	{
		syncFaceFromRotation();
		setHome();
	}

	~Nav4D() = default;

	// -------- Smoothing (Nav-style) --------

	/// Set smoothing in [0,1). Larger -> more inertia.
	Nav4D &smooth(float v)
	{
		mSmooth = v;
		return *this;
	}

	float smooth() const { return mSmooth; }

	// -------- Local-axis movement controls --------

	/// Halt both raw and smoothed velocities, and clear one-shot increments.
	void halt()
	{
		mMove0.set(0);
		mMove1.set(0);
		mNudge.set(0);

		for (int i = 0; i < 6; ++i)
		{
			mSpin0[i] = 0.f;
			mSpin1[i] = 0.f;
			mTurn[i] = 0.f;
		}
	}

	/// Store current transform as home.
	void setHome()
	{
		mHomePos = pos;
		mHomeRot = rotationState;
	}

	/// Reset to home immediately.
	void home()
	{
		pos = mHomePos;
		rotationState = mHomeRot;
		syncFaceFromRotation();
	}

	/// Set linear velocity along local axis.
	/// axis: 0=x (right), 1=y (up), 2=z (backward), 3=w (kata).
	void moveLocal(int axis, float v)
	{
		if (axis < 0 || axis > 3)
		{
			return;
		}
		mMove0[axis] = v;
	}

	/// Set linear velocity in local coordinates (x,right; y,up; z,backward; w,kata).
	void moveLocal(const Vec4f &v) { mMove0 = v; }

	/// One-shot displacement increment (applied once at next step).
	void nudgeLocal(int axis, float amount)
	{
		if (axis < 0 || axis > 3)
		{
			return;
		}
		mNudge[axis] += amount;
	}

	void nudgeLocal(const Vec4f &delta) { mNudge += delta; }

	// -------- 6-plane rotational controls --------

	/// Plane index mapping used by Rotation4D::fromRates inputs:
	/// 0=XY, 1=YZ, 2=ZX, 3=XW, 4=YW, 5=ZW
	enum Plane6
	{
		XY = 0,
		YZ = 1,
		ZX = 2,
		XW = 3,
		YW = 4,
		ZW = 5
	};

	/// Set angular velocity rate (rad/sec) for a given cardinal plane in local coordinates.
	void spinPlane(Plane6 plane, float radPerSec) { mSpin0[(int)plane] = radPerSec; }

	/// Set one-shot angular increment (radians) for the next step in that plane.
	void turnPlane(Plane6 plane, float radIncrement) { mTurn[(int)plane] = radIncrement; }

	/// Convenience: set all 6 plane rates at once.
	void spinPlanes(float rXY, float rYZ, float rZX, float rXW, float rYW, float rZW)
	{
		mSpin0 = {rXY, rYZ, rZX, rXW, rYW, rZW};
	}

	// -------- Integration --------

	/// Integrate velocities over dt and update 4D camera pose.
	void step(double dt)
	{
		const float fdt = static_cast<float>(dt);
		if (fdt == 0.f)
		{
			return;
		}

		// Low-pass filter like al::Nav.
		const float amt = 1.f - mSmooth;

		// Translation: mMove1 becomes smoothed displacement for this step.
		// mMove0 is velocity in local coordinates; multiply by dt -> displacement.
		mMove1.lerp(mMove0 * fdt + mNudge, amt);

		// Rotation: mSpin1 becomes smoothed plane angle increments for this step.
		for (int i = 0; i < 6; ++i)
		{
			const float target = mSpin0[i] * fdt + mTurn[i];
			mSpin1[i] = mSpin1[i] + (target - mSpin1[i]) * amt;
		}

		// Clear one-shot increments after converting them into this step's targets.
		mNudge.set(0);
		for (int i = 0; i < 6; ++i)
		{
			mTurn[i] = 0.f;
		}

		// Apply orientation delta in local frame.
		const Rotation4D delta = Rotation4D::fromRates(
			mSpin1[0],
			mSpin1[1],
			mSpin1[2],
			mSpin1[3],
			mSpin1[4],
			mSpin1[5]);
		applyRotation(delta);
		syncFaceFromRotation();

		// Move along the updated local basis.
		pos += axisToWorld(0) * mMove1[0];
		pos += axisToWorld(1) * mMove1[1];
		pos += axisToWorld(2) * mMove1[2];
		pos += axisToWorld(3) * mMove1[3];
	}

	// -------- World <-> viewer-local transforms --------

	/// Convert a world point into viewer-local coordinates:
	/// p_local = R⁻¹ · (p_world − C)
	Vec4f toLocal(const Vec4f &pWorld) const
	{
		const Rotation4D inv = rotationState.inverse();
		return inv.apply(pWorld - pos);
	}

	/// Convert viewer-local coordinates back to world:
	/// p_world = C + R · p_local
	Vec4f toWorld(const Vec4f &pLocal) const
	{
		return pos + rotationState.apply(pLocal);
	}

	/// Get local basis directions expressed in world coordinates.
	/// Axes match toLocal() output:
	/// - axis 0: x right  -> faceDirection.faceRight
	/// - axis 1: y up     -> faceDirection.faceUp
	/// - axis 2: z backward -> -faceDirection.faceForward
	/// - axis 3: w kata   -> -faceDirection.faceAna
	Vec4f axisToWorld(int axis) const
	{
		switch (axis)
		{
		case 0:
			return faceDirection.faceRight.normalized();
		case 1:
			return faceDirection.faceUp.normalized();
		case 2:
			return (-faceDirection.faceForward).normalized();
		case 3:
			return (-faceDirection.faceAna).normalized();
		default:
			return Vec4f(0.f, 0.f, 0.f, 0.f);
		}
	}

	Nav4D(const Nav4D &) = default;
	Nav4D &operator=(const Nav4D &) = default;


};