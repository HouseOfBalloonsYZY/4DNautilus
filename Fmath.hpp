#pragma once

#include "al/math/al_Quat.hpp"
#include "al/math/al_Vec.hpp"
#include <cmath>

using namespace al;

// ---------------------------------------------------------------------------
// 4D vectors (x, y, z, w). Float for performance with many calculations.
//
// Axis convention (matches allolib/OpenGL for x,y,z; w is the fourth axis):
//   x = right
//   y = up
//   z = backward (into the screen)
//   w = toward the viewer / into the parallel 3D world "in front" of the viewer
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Vec4 <-> Quat (for 4D rotation formula)
// Convention: Vec4f(x,y,z,w) <-> Quatf(w,x,y,z) so R^4 = quaternions.
// ---------------------------------------------------------------------------

inline Quatf vec4ToQuat(const Vec4f &v)
{
    return Quatf(v.w, v.x, v.y, v.z);
}

inline Vec4f quatToVec4(const Quatf &q)
{
    return Vec4f(q.x, q.y, q.z, q.w);
}

// ---------------------------------------------------------------------------
// Rotation4D: 4D rotation stored as a pair of unit quaternions (qL, qR).
// Action on v:  v' = qL * v * conj(qR)  (v as quaternion).
// Uses only allolib Quat and Vec4; no matrices.
//
// Isoclinic vs double rotation (see Wikipedia: Rotations in 4-dimensional
// Euclidean space): A left-isoclinic rotation uses only qL (qR = identity);
// a right-isoclinic uses only qR (qL = identity). Every 4D rotation in SO(4)
// is uniquely (up to sign) the composition of one left- and one right-isoclinic,
// so arbitrary (qL, qR) is correct and covers all 4D rotations, including
// double rotations (two different angles in two invariant planes). Use
// fromLeftQuat/fromRightQuat when you want a single isoclinic rotation.
// ---------------------------------------------------------------------------

struct Rotation4D
{
    // Identity rotation (no rotation): w=1, x=y=z=0 for both quaternions.
    // Default for all 4D objects until rotated.
    Quatf qL{Quatf::identity()};
    Quatf qR{Quatf::identity()};

    Rotation4D() = default;

    Rotation4D(const Quatf &left, const Quatf &right)
        : qL(left), qR(right)
    {
        qL.normalize();
        qR.normalize();
    }

    static Rotation4D identity()
    {
        return Rotation4D();
    }

    /** Rotation by left-multiplication only (qR = 1). */
    static Rotation4D fromLeftQuat(const Quatf &q)
    {
        Rotation4D r;
        r.qL = q;
        r.qL.normalize();
        return r;
    }

    /** Rotation by right-multiplication only (qL = 1). */
    static Rotation4D fromRightQuat(const Quatf &q)
    {
        Rotation4D r;
        r.qR = q;
        r.qR.normalize();
        return r;
    }

    /** Simple rotation in one of the 6 coordinate planes: exactly one plane
     *  rotates, the orthogonal 2-plane stays fixed. axis1, axis2 in {0,1,2,3}
     *  for x,y,z,w; angle in radians.
     *
     *  Rule A — spatial planes (xy, xz, yz): qR = conj(qL) so the w-axis
     *  (and the axis perpendicular to the plane) stay fixed.
     *  Rule B — planes involving w (xw, yw, zw): qR = qL so the two spatial
     *  axes not in the plane stay fixed. Both cases handled below. */
    static Rotation4D fromPlaneAngle(int axis1, int axis2, float angleRad)
    {
        if (axis1 > axis2)
        {
            std::swap(axis1, axis2);
        }
        // Plane index 0..5 for (01), (02), (03), (12), (13), (23)
        const int planeIndex = axis1 * (7 - axis1) / 2 + axis2 - axis1 - 1;
        if (planeIndex < 0 || planeIndex > 5)
        {
            return Rotation4D::identity();
        }
        const float half = angleRad * 0.5f;
        const float c = std::cos(half);
        const float s = std::sin(half);
        // Quatf(w,x,y,z) = (scalar, i, j, k). Which imaginary axis (1=i, 2=j, 3=k)
        // drives this plane: xy->k, xz->j, xw->i, yz->i, yw->j, zw->k.
        static const int quatAxis[6] = {3, 2, 1, 1, 2, 3};
        const int qa = quatAxis[planeIndex];
        Quatf qL(c, qa == 1 ? s : 0.f, qa == 2 ? s : 0.f, qa == 3 ? s : 0.f);
        // Rule B: plane contains w (axis 3) -> qR = qL. Rule A: else qR = conj(qL).
        const bool planeContainsW = (axis1 == 3 || axis2 == 3);
        Quatf qR = planeContainsW ? qL : qL.conj();
        return Rotation4D(qL, qR);
    }

    // --- Wedge-product / SU(2) construction (see quaternion4DRotation.md) ---

    /** Build a unit quaternion from a 3D "axis-angle" vector via exponential map:
     *  q = cos(|v|) + sin(|v|) * v/|v|. Returns identity if |v| is negligible.
     *  - axis = (vx,vy,vz) is the generator vector (abstract i,j,k components).
     *  - theta = |axis| is the rotation half-angle; axis/theta is the unit axis.
     *  - eps: if |axis| is nearly zero we skip division and return identity. */
    static Quatf quatFromImaginary(float vx, float vy, float vz)
    {
        const float eps = 1e-7f;
        Vec3f axis(vx, vy, vz);
        float theta = axis.mag();
        if (theta < eps)
        {
            return Quatf::identity();
        }
        Quatf q;
        q.fromAxisAngle(theta, axis / theta);
        q.normalize();
        return q;
    }

    /** Build rotation from the 6 cardinal plane rates (wedge decomposition).
     *  vL = (R_yz+R_xw, R_zx+R_yw, R_xy+R_zw), vR = (-R_yz+R_xw, -R_zx+R_yw, -R_xy+R_zw);
     *  then qL = exp(vL), qR = conj(exp(vR)) so apply is qL*v*conj(qR). */
    static Rotation4D fromRates(float rXY, float rYZ, float rZX, float rXW, float rYW, float rZW)
    {
        Quatf leftQuat = quatFromImaginary(rYZ + rXW, rZX + rYW, rXY + rZW);
        Quatf rightQuat = quatFromImaginary(-rYZ + rXW, -rZX + rYW, -rXY + rZW);
        return Rotation4D(leftQuat, rightQuat.conj());
    }

    /** Rotation in the plane spanned by two 4D vectors u, v by angle (radians).
     *  Wedge product gives the 6 rates, then fromRates. u,v need not be unit. */
    static Rotation4D fromPlane(const Vec4f &u, const Vec4f &v, float angleRad)
    {
        Vec4f u_normalized = u.normalized();
        Vec4f v_normalized = v.normalized();
        float rXY = (u_normalized.x * v_normalized.y - u_normalized.y * v_normalized.x) * angleRad;
        float rYZ = (u_normalized.y * v_normalized.z - u_normalized.z * v_normalized.y) * angleRad;
        float rZX = (u_normalized.z * v_normalized.x - u_normalized.x * v_normalized.z) * angleRad;
        float rXW = (u_normalized.x * v_normalized.w - u_normalized.w * v_normalized.x) * angleRad;
        float rYW = (u_normalized.y * v_normalized.w - u_normalized.w * v_normalized.y) * angleRad;
        float rZW = (u_normalized.z * v_normalized.w - u_normalized.w * v_normalized.z) * angleRad;
        return fromRates(rXY, rYZ, rZX, rXW, rYW, rZW);
    }

    /** Double rotation: two orthogonal planes with angles. Rates add linearly. */
    static Rotation4D fromTwoPlanes(
        const Vec4f &u1, const Vec4f &v1, float angle1Rad,
        const Vec4f &u2, const Vec4f &v2, float angle2Rad)
    {
        Vec4f u1_normalized = u1.normalized();
        Vec4f v1_normalized = v1.normalized();
        Vec4f u2_normalized = u2.normalized();
        Vec4f v2_normalized = v2.normalized();
        float rXY = (u1_normalized.x * v1_normalized.y - u1_normalized.y * v1_normalized.x) * angle1Rad + (u2_normalized.x * v2_normalized.y - u2_normalized.y * v2_normalized.x) * angle2Rad;
        float rYZ = (u1_normalized.y * v1_normalized.z - u1_normalized.z * v1_normalized.y) * angle1Rad + (u2_normalized.y * v2_normalized.z - u2_normalized.z * v2_normalized.y) * angle2Rad;
        float rZX = (u1_normalized.z * v1_normalized.x - u1_normalized.x * v1_normalized.z) * angle1Rad + (u2_normalized.z * v2_normalized.x - u2_normalized.x * v2_normalized.z) * angle2Rad;
        float rXW = (u1_normalized.x * v1_normalized.w - u1_normalized.w * v1_normalized.x) * angle1Rad + (u2_normalized.x * v2_normalized.w - u2_normalized.w * v2_normalized.x) * angle2Rad;
        float rYW = (u1_normalized.y * v1_normalized.w - u1_normalized.w * v1_normalized.y) * angle1Rad + (u2_normalized.y * v2_normalized.w - u2_normalized.w * v2_normalized.y) * angle2Rad;
        float rZW = (u1_normalized.z * v1_normalized.w - u1_normalized.w * v1_normalized.z) * angle1Rad + (u2_normalized.z * v2_normalized.w - u2_normalized.w * v2_normalized.z) * angle2Rad;
        return fromRates(rXY, rYZ, rZX, rXW, rYW, rZW);
    }

    /** Accumulate a new rotation on top of this (new applied after current).
     *  qL' = newRot.qL * qL, qR' = qR * newRot.qR. */
    void prepend(const Rotation4D &newRot)
    {
        qL = newRot.qL.multiply(qL);
        qR = qR.multiply(newRot.qR);

        qL.normalize();
        qR.normalize();
    }

    /** Rotate 4D vector: v' = qL * v * conj(qR). Same as (1) left multiply by qL,
     *  then (2) right multiply by conj(qR); we store qR so the formula uses conj(qR). */
    Vec4f apply(const Vec4f &v) const
    {
        Quatf p = vec4ToQuat(v);
        p = qL.multiply(p).multiply(qR.conj());
        return quatToVec4(p);
    }

    Vec4f operator()(const Vec4f &v) const
    {
        return apply(v);
    }

    /** Compose rotations: (this * other).apply(v) == this.apply(other.apply(v)). */
    Rotation4D operator*(const Rotation4D &other) const
    {
        return Rotation4D(qL.multiply(other.qL).normalize(), qR.multiply(other.qR).normalize().normalize());
    }

    Rotation4D &operator*=(const Rotation4D &other)
    {
        qL = qL.multiply(other.qL);
        qR = qR.multiply(other.qR);
        qL.normalize();
        qR.normalize();
        return *this;
    }

    /** Inverse: apply then inverse gives identity. */
    Rotation4D inverse() const
    {
        return Rotation4D(qL.conj(), qR.conj());
    }
};

// ---------------------------------------------------------------------------
// Spherical linear interpolation between two 4D rotations (slerp on both
// quaternions using allolib's Quat::slerp).
// ---------------------------------------------------------------------------

inline Rotation4D slerp(const Rotation4D &from, const Rotation4D &to, float t)
{
    return Rotation4D(
        Quatf::slerp(from.qL, to.qL, t),
        Quatf::slerp(from.qR, to.qR, t));
}
