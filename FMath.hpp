#pragma once

#include "al/math/al_Quat.hpp"
#include "al/math/al_Vec.hpp"
#include <cmath>
#include <array>

using namespace al;

// ------------------------------------------------------------
// helpers
// ------------------------------------------------------------

inline Quatf vec4ToQuat(const Vec4f &v) { return Quatf(v.w, v.x, v.y, v.z); }

inline Vec4f quatToVec4(const Quatf &q) { return Vec4f(q.x, q.y, q.z, q.w); }

inline float degreesToRad(float angleDeg)
{
    float a = std::fmod(angleDeg, 360.f);
    if (a > 180.f)
    {
        a -= 360.f;
    }
    return a * (3.14159265358979f / 180.f);
}

inline float clamp(float v, float maxAbs)
{
    return std::max(-maxAbs, std::min(maxAbs, v));
}

// Orthonormalize a spanning pair for a 2-plane (Gram-Schmidt).
// Returns false if u is near-zero or u,v are near-parallel.
inline bool orthonormalizePlane(Vec4f &u, Vec4f &v)
{
    const float eps = 1e-7f;
    if (u.mag() < eps)
    {
        return false;
    }
    u = u.normalized();
    v = v - u * v.dot(u);
    if (v.mag() < eps)
    {
        return false;
    }
    v = v.normalized();
    return true;
}

inline Vec4f clampVec4(const Vec4f &v, float maxAbs)
{
    Vec4f result = v;
    result.x = clamp(result.x, maxAbs);
    result.y = clamp(result.y, maxAbs);
    result.z = clamp(result.z, maxAbs);
    result.w = clamp(result.w, maxAbs);
    return result;
}

inline std::array<float, 6> clampArray6(const std::array<float, 6> &rates, float maxAbs)
{
    std::array<float, 6> result = rates;
    for (int i = 0; i < 6; ++i)
    {
        result[i] = clamp(result[i], maxAbs);
    }
    return result;
}

inline std::array<float, 6> addArray6(const std::array<float, 6> &array1, const std::array<float, 6> &array2)
{
    std::array<float, 6> result = array1;
    for (int i = 0; i < 6; ++i)
    {
        result[i] += array2[i];
    }
    return result;
}

// ------------------------------------------------------------

struct FaceDirection;

struct Rotation4D
{
private:
    // Second SU(2) factor (qR) in the Spin(4) pair (qL, qR) is Stored as exp(vR) from
    // wedge/rates (Rotation.md ApplyRotation). apply() uses this directly as the
    // right multiplier: V' = qL * V * qR.
    Quatf qL{Quatf::identity()};
    Quatf qR{Quatf::identity()};

    void setQL(const Quatf &q)
    {
        qL = q;
        qL.normalize();
    }
    void setQR(const Quatf &q)
    {
        qR = q;
        qR.normalize();
    }

    /** Continue building rotation from the 6 cardinal plane rates (wedge decomposition).
     *  vL = (R_yz+R_xw, R_zx+R_yw, R_xy+R_zw), vR = (-R_yz+R_xw, -R_zx+R_yw,
     * -R_xy+R_zw); then qL = exp(vL), qR = exp(vR). Same as Rotation.md
     * ApplyRotation. */
    // Plane-rate order convention (project-wide):
    // (rXY, rXZ, rXW, rYZ, rYW, rZW)
    static Rotation4D fromRates(float rXY, float rXZ, float rXW, float rYZ, float rYW, float rZW)
    {
        // Internal construction expects bivector components in the order used by the Spin(4)
        // (qL,qR) exponential map. This matches the prior implementation if we substitute:
        // rZX(old) = -rXZ(new)
        const float rZX = -rXZ;
        Quatf leftQuat = quatFromImaginary(rYZ + rXW, rZX + rYW, rXY + rZW);
        Quatf rightQuat = quatFromImaginary(-rYZ + rXW, -rZX + rYW, -rXY + rZW);
        return Rotation4D(leftQuat, rightQuat);
    }

    /** Continue building a unit quaternion from a 3D "axis-angle" vector via exponential map:
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

        const float inv = 1.f / theta;
        const float s = std::sin(theta);
        const float c = std::cos(theta);
        Quatf q(c, axis.x * inv * s, axis.y * inv * s, axis.z * inv * s);
        q.normalize();
        return q;
    }

public:
    const Quatf &getQL() const { return qL; }
    const Quatf &getQR() const { return qR; }

    Rotation4D() = default;

    Rotation4D(const Quatf &left, const Quatf &right)
    {
        setQL(left);
        setQR(right);
    }

    // additional factory functions
    // identical rotation
    static Rotation4D identity() { return Rotation4D(); }

    // Rotation by left-multiplication only (qR = 1).
    static Rotation4D fromLeftQuat(const Quatf &q)
    {
        Rotation4D r;
        r.qL = q;
        r.normalize();
        return r;
    }

    // Rotation by right-multiplication only (qL = 1).
    static Rotation4D fromRightQuat(const Quatf &q)
    {
        Rotation4D r;
        r.qR = q;
        r.normalize();
        return r;
    }

    Rotation4D& normalize()
    {
        qL.normalize();
        qR.normalize();
        return *this;
    }

    // constructors

    /** Simple rotation in one of the 6 coordinate planes: exactly one plane
     *  rotates, the orthogonal 2-plane stays fixed. axis1, axis2 in {0,1,2,3}
     *  for x,y,z,w; angle in radians.
     *
     *  Rule A — spatial planes (xy, xz, yz): qR = qL so the w-axis stays fixed.
     *  Rule B — planes involving w (xw, yw, zw): qR = conj(qL) so the two spatial
     *  axes not in the plane stay fixed. */
    static Rotation4D fromGlobalPlane(int axis1, int axis2, float angleRad)
    {
        if (axis1 > axis2)
        {
            std::swap(axis1, axis2);
        }

        if (axis1 == axis2)
        {
            return Rotation4D::identity();
        }

        if (axis1 < 0 || axis1 > 3 || axis2 < 0 || axis2 > 3)
        {
            return Rotation4D::identity();
        }

        const float halfAngle = angleRad * 0.5f;
        const float cosHalf = std::cos(halfAngle);
        const float sinHalf = std::sin(halfAngle);

        // Quatf(w, x, y, z) = scalar + i + j + k (see vec4ToQuat).
        Quatf qL;
        Quatf qR;

        const bool isWPlane = (axis2 == 3);

        if (axis1 == 0)
        {
            if (axis2 == 1)
            {
                // XY plane: rotation generator is k (imaginary z / quat .z).
                qL = Quatf(cosHalf, 0.f, 0.f, sinHalf);
            }
            else if (axis2 == 2)
            {
                // XZ plane: generator is j (quat .y).
                qL = Quatf(cosHalf, 0.f, sinHalf, 0.f);
            }
            else if (axis2 == 3)
            {
                // XW plane: generator is i (quat .x).
                qL = Quatf(cosHalf, sinHalf, 0.f, 0.f);
            }
            else
            {
                return Rotation4D::identity();
            }
        }
        else if (axis1 == 1)
        {
            if (axis2 == 2)
            {
                // YZ plane: generator is i (quat .x).
                qL = Quatf(cosHalf, sinHalf, 0.f, 0.f);
            }
            else if (axis2 == 3)
            {
                // YW plane: generator is j (quat .y).
                qL = Quatf(cosHalf, 0.f, sinHalf, 0.f);
            }
            else
            {
                return Rotation4D::identity();
            }
        }
        else if (axis1 == 2)
        {
            if (axis2 == 3)
            {
                // ZW plane: generator is k (quat .z).
                qL = Quatf(cosHalf, 0.f, 0.f, sinHalf);
            }
            else
            {
                return Rotation4D::identity();
            }
        }
        else
        {
            return Rotation4D::identity();
        }

        if (isWPlane)
        {
            // Rule B: xw, yw, zw — keep the two spatial axes outside the plane fixed.
            qR = qL.conj();
        }
        else
        {
            // Rule A: xy, xz, yz — keep w fixed.
            qR = qL;
        }

        return Rotation4D(qL, qR);
    }

    /** Simple rotation in the plane span(u,v) by angleRad (geometric radians).
     *  Orthonormalizes u,v (Gram-Schmidt), builds unit wedge rates at half-angle
     *  (sandwich uses q=cos(α)+… with geometric angle 2α), then fromRates. */
    // Plane-rate order convention (project-wide):
    // (rXY, rXZ, rXW, rYZ, rYW, rZW)
    static Rotation4D fromPlane(const Vec4f &u, const Vec4f &v, float angleRad)
    {
        Vec4f uOrtho = u;
        Vec4f vOrtho = v;
        if (!orthonormalizePlane(uOrtho, vOrtho))
        {
            return Rotation4D::identity();
        }

        // Half-angle: |v_L| must be geometricAngle/2 for correct sandwich magnitude.
        const float half = angleRad * 0.5f;
        float rXY = (uOrtho.x * vOrtho.y - uOrtho.y * vOrtho.x) * half;
        // Note: rXZ = -rZX (antisymmetry of the wedge / bivector components).
        float rXZ = (uOrtho.x * vOrtho.z - uOrtho.z * vOrtho.x) * half;
        float rXW = (uOrtho.x * vOrtho.w - uOrtho.w * vOrtho.x) * half;
        float rYZ = (uOrtho.y * vOrtho.z - uOrtho.z * vOrtho.y) * half;
        float rYW = (uOrtho.y * vOrtho.w - uOrtho.w * vOrtho.y) * half;
        float rZW = (uOrtho.z * vOrtho.w - uOrtho.w * vOrtho.z) * half;
        return fromRates(rXY, rXZ, rXW, rYZ, rYW, rZW);
    }

    /** Double rotation in two planes (ideally orthogonal). Same half-angle + Gram-Schmidt
     *  per plane as fromPlane; rates add linearly. */
    static Rotation4D fromPlanes(const Vec4f &u1, const Vec4f &v1, float angle1Rad,
                                 const Vec4f &u2, const Vec4f &v2, float angle2Rad)
    {
        Vec4f a1 = u1;
        Vec4f b1 = v1;
        Vec4f a2 = u2;
        Vec4f b2 = v2;
        const bool ok1 = orthonormalizePlane(a1, b1);
        const bool ok2 = orthonormalizePlane(a2, b2);
        if (!ok1 && !ok2)
        {
            return Rotation4D::identity();
        }

        const float h1 = ok1 ? (angle1Rad * 0.5f) : 0.f;
        const float h2 = ok2 ? (angle2Rad * 0.5f) : 0.f;

        float rXY = (a1.x * b1.y - a1.y * b1.x) * h1 + (a2.x * b2.y - a2.y * b2.x) * h2;
        float rXZ = (a1.x * b1.z - a1.z * b1.x) * h1 + (a2.x * b2.z - a2.z * b2.x) * h2;
        float rXW = (a1.x * b1.w - a1.w * b1.x) * h1 + (a2.x * b2.w - a2.w * b2.x) * h2;
        float rYZ = (a1.y * b1.z - a1.z * b1.y) * h1 + (a2.y * b2.z - a2.z * b2.y) * h2;
        float rYW = (a1.y * b1.w - a1.w * b1.y) * h1 + (a2.y * b2.w - a2.w * b2.y) * h2;
        float rZW = (a1.z * b1.w - a1.w * b1.z) * h1 + (a2.z * b2.w - a2.w * b2.z) * h2;
        return fromRates(rXY, rXZ, rXW, rYZ, rYW, rZW);
    }

    // this has to be implemented outside becasue FaceDirection hasn't been defined yet at this point in the file
    static Rotation4D fromLocalPlane(const FaceDirection &face, int axis1, int axis2, float angleRad);

    /// Accumulate this frame's rotation after the current state (Plan A / face-plane deltas).
    /// R_total = delta ∘ R_old  →  qL = qL_delta·qL_old,  qR = qR_old·qR_delta
    void append(const Rotation4D &delta)
    {
        qL = delta.qL.multiply(qL);
        qR = qR.multiply(delta.qR);

        qL.normalize();
        qR.normalize();
    }

    Vec4f apply(const Vec4f &v) const
    {
        Quatf p = vec4ToQuat(v);
        p = qL.multiply(p).multiply(qR);
        return quatToVec4(p);
    }

    Vec4f operator()(const Vec4f &v) const { return apply(v); }

    Rotation4D inverse() const { return Rotation4D(qL.conj(), qR.conj()); }

    static Rotation4D slerp(const Rotation4D &from, const Rotation4D &to, float t)
    {
        return Rotation4D(Quatf::slerp(from.getQL(), to.getQL(), t),
                          Quatf::slerp(from.getQR(), to.getQR(), t));
    }
};

struct FaceDirection
{
    std::array<Vec4f, 4> face =
        {
            Vec4f(1.f, 0.f, 0.f, 0.f),
            Vec4f(0.f, 1.f, 0.f, 0.f),
            Vec4f(0.f, 0.f, -1.f, 0.f),
            Vec4f(0.f, 0.f, 0.f, -1.f)};

    FaceDirection() = default;

    void updateFaceDirection(Rotation4D &rotation)
    {
        face[0] = rotation.apply(Vec4f(1.f, 0.f, 0.f, 0.f)).normalized();
        face[1] = rotation.apply(Vec4f(0.f, 1.f, 0.f, 0.f)).normalized();
        face[2] = rotation.apply(Vec4f(0.f, 0.f, -1.f, 0.f)).normalized();
        face[3] = rotation.apply(Vec4f(0.f, 0.f, 0.f, -1.f)).normalized();
    }
};

inline Rotation4D Rotation4D::fromLocalPlane(const FaceDirection &face, int axis1, int axis2, float angleRad)
{
    if (axis1 < 0 || axis1 > 3 || axis2 < 0 || axis2 > 3)
    {
        return Rotation4D::identity();
    }

    if (axis1 == axis2)
    {
        return Rotation4D::identity();
    }

    if (axis1 > axis2)
    {
        std::swap(axis1, axis2);
        angleRad = -angleRad;
    }

    if (axis1 == 0 || axis1 == 1)
    {
        if (axis2 == 0 || axis2 == 1)
        {
            return Rotation4D::fromPlane(face.face[axis1], face.face[axis2], angleRad);
        }
        else
        {
            return Rotation4D::fromPlane(face.face[axis1], -face.face[axis2], angleRad);
        }
    }
    else if (axis1 == 2 || axis1 == 3)
    {
        if (axis2 == 0 || axis2 == 1)
        {
            return Rotation4D::fromPlane(-face.face[axis1], face.face[axis2], angleRad);
        }
        else
        {
            return Rotation4D::fromPlane(-face.face[axis1], -face.face[axis2], angleRad);
        }
    }
}