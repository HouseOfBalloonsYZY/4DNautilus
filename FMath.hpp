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

// Complete (u,v) to a positively oriented ONB (u,v,n1,n2).
inline bool completePlaneBasis(Vec4f &u, Vec4f &v, Vec4f &n1, Vec4f &n2)
{
    if (!orthonormalizePlane(u, v))
    {
        return false;
    }

    const Vec4f candidates[4] = {
        Vec4f(1.f, 0.f, 0.f, 0.f),
        Vec4f(0.f, 1.f, 0.f, 0.f),
        Vec4f(0.f, 0.f, 1.f, 0.f),
        Vec4f(0.f, 0.f, 0.f, 1.f)};

    auto makeOrtho = [&](const Vec4f *basis, int basisCount, Vec4f &out) -> bool
    {
        for (const Vec4f &e : candidates)
        {
            Vec4f n = e;
            for (int i = 0; i < basisCount; ++i)
            {
                n = n - basis[i] * n.dot(basis[i]);
            }
            if (n.mag() > 1e-7f)
            {
                out = n.normalized();
                return true;
            }
        }
        return false;
    };

    Vec4f basis2[2] = {u, v};
    if (!makeOrtho(basis2, 2, n1))
    {
        return false;
    }
    Vec4f basis3[3] = {u, v, n1};
    if (!makeOrtho(basis3, 3, n2))
    {
        return false;
    }

    // det[u v n1 n2] via scalar quadruple product; flip n2 if negatively oriented.
    // For columns c0..c3, det = ε_ijkl c0_i c1_j c2_k c3_l.
    float det = 0.f;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (j == i)
            {
                continue;
            }
            for (int k = 0; k < 4; ++k)
            {
                if (k == i || k == j)
                {
                    continue;
                }
                for (int l = 0; l < 4; ++l)
                {
                    if (l == i || l == j || l == k)
                    {
                        continue;
                    }
                    // parity of permutation (i,j,k,l)
                    int p[4] = {i, j, k, l};
                    int invs = 0;
                    for (int a = 0; a < 4; ++a)
                    {
                        for (int b = a + 1; b < 4; ++b)
                        {
                            if (p[a] > p[b])
                            {
                                ++invs;
                            }
                        }
                    }
                    const float sign = (invs % 2 == 0) ? 1.f : -1.f;
                    det += sign * u[i] * v[j] * n1[k] * n2[l];
                }
            }
        }
    }
    if (det < 0.f)
    {
        n2 = -n2;
    }
    return true;
}

// Unit quat with q * a * conj(q) = b for unit 3-vectors a,b (pure imaginaries).
inline Quatf quatAlignVec3(const Vec3f &aIn, const Vec3f &bIn)
{
    Vec3f a = aIn.normalized();
    Vec3f b = bIn.normalized();
    const float d = a.dot(b);
    if (d < -1.f + 1e-6f)
    {
        Vec3f axis = (std::fabs(a.x) < 0.9f) ? a.cross(Vec3f(1.f, 0.f, 0.f))
                                             : a.cross(Vec3f(0.f, 1.f, 0.f));
        axis = axis.normalized();
        return Quatf(0.f, axis.x, axis.y, axis.z);
    }
    const Vec3f c = a.cross(b);
    Quatf q(1.f + d, c.x, c.y, c.z);
    q.normalize();
    return q;
}

// q such that sandwich maps i→iTo, j→jTo in imaginary 3-space.
inline Quatf quatAlignImagFrame(const Vec3f &iTo, const Vec3f &jTo)
{
    const Quatf q1 = quatAlignVec3(Vec3f(1.f, 0.f, 0.f), iTo);
    // j' = q1 * j * conj(q1)
    const Quatf jPure(0.f, 0.f, 1.f, 0.f);
    const Quatf jpQ = q1.multiply(jPure).multiply(q1.conj());
    const Vec3f jp(jpQ.x, jpQ.y, jpQ.z);
    const Quatf q2 = quatAlignVec3(jp, jTo);
    Quatf q = q2.multiply(q1);
    q.normalize();
    return q;
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

    /** Simple rotation in plane span(u,v) by angleRad (geometric radians).
     *  Built as M ∘ L_xy ∘ M^{-1}: L_xy is the known-good cardinal XY spin;
     *  M maps (ex,ey) → orthonormal (u,v). Matches Rodrigues (complement fixed).
     *  (Wedge→fromRates is NOT used here — it fails for tilted planes.) */
    static Rotation4D fromPlane(const Vec4f &uIn, const Vec4f &vIn, float angleRad)
    {
        Vec4f u = uIn;
        Vec4f v = vIn;
        Vec4f n1;
        Vec4f n2;
        if (!completePlaneBasis(u, v, n1, n2))
        {
            return Rotation4D::identity();
        }

        // M: apply sends ex,ey,ez,ew → u,v,n1,n2
        const Quatf U = vec4ToQuat(u);
        const Quatf V = vec4ToQuat(v);
        const Quatf One = vec4ToQuat(n2); // ew → scalar 1
        const Quatf ti = U.multiply(One.conj());
        const Quatf tj = V.multiply(One.conj());
        const Quatf qLm = quatAlignImagFrame(Vec3f(ti.x, ti.y, ti.z), Vec3f(tj.x, tj.y, tj.z));
        Quatf qRm = qLm.conj().multiply(One);
        qRm.normalize();
        const Rotation4D M(qLm, qRm);

        // L: simple XY by angleRad (qR = conj(qL) → fixes z,w)
        const float ha = angleRad * 0.5f;
        const float c = std::cos(ha);
        const float s = std::sin(ha);
        const Rotation4D L(Quatf(c, 0.f, 0.f, s), Quatf(c, 0.f, 0.f, -s));

        // W = M ∘ L ∘ M^{-1}
        const Rotation4D Mi = M.inverse();
        const Rotation4D LMi(L.getQL().multiply(Mi.getQL()), Mi.getQR().multiply(L.getQR()));
        return Rotation4D(M.getQL().multiply(LMi.getQL()), LMi.getQR().multiply(M.getQR()));
    }

    /** Double rotation: compose two simple fromPlane spins (exact when planes are orthogonal). */
    static Rotation4D fromPlanes(const Vec4f &u1, const Vec4f &v1, float angle1Rad,
                                 const Vec4f &u2, const Vec4f &v2, float angle2Rad)
    {
        const Rotation4D r1 = fromPlane(u1, v1, angle1Rad);
        const Rotation4D r2 = fromPlane(u2, v2, angle2Rad);
        // r1 ∘ r2
        return Rotation4D(r1.getQL().multiply(r2.getQL()), r2.getQR().multiply(r1.getQR()));
    }

    // this has to be implemented outside becasue FaceDirection hasn't been defined yet at this point in the file
    static Rotation4D fromLocalPlane(const FaceDirection &face, int axis1, int axis2, float angleRad);

    /// Cardinal simple spin in the body axis convention (+X,+Y,-Z,-W), same signs as default FaceDirection.
    /// Used for body-local compose R ← R ∘ L (no face-dependent M).
    static Rotation4D fromBodyPlane(int axis1, int axis2, float angleRad);

    /// Accumulate this frame's rotation after the current state (Plan A / face-plane deltas).
    /// R_total = delta ∘ R_old  →  qL = qL_delta·qL_old,  qR = qR_old·qR_delta
    void append(const Rotation4D &delta)
    {
        qL = delta.qL.multiply(qL);
        qR = qR.multiply(delta.qR);

        qL.normalize();
        qR.normalize();
    }

    /// Body-local compose: R ← R ∘ L
    /// (apply L in the object's current body frame, then existing R).
    /// qL' = qL qL_L,  qR' = qR_L qR
    void composeRight(const Rotation4D &L)
    {
        qL = qL.multiply(L.qL);
        qR = L.qR.multiply(qR);

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
        return Rotation4D::fromPlane(face.face[axis1], -face.face[axis2], angleRad);
    }

    // axis1 is 2 or 3
    if (axis2 == 0 || axis2 == 1)
    {
        return Rotation4D::fromPlane(-face.face[axis1], face.face[axis2], angleRad);
    }
    return Rotation4D::fromPlane(-face.face[axis1], -face.face[axis2], angleRad);
}

inline Rotation4D Rotation4D::fromBodyPlane(int axis1, int axis2, float angleRad)
{
    // Fixed body axes matching default FaceDirection (+X,+Y,-Z,-W).
    // fromPlane on these is pose-independent → no M singularity while navigating.
    static const FaceDirection kBodyAxes{};
    return fromLocalPlane(kBodyAxes, axis1, axis2, angleRad);
}