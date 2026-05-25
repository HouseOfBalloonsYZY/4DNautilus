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

inline Quatf vec4ToQuat(const Vec4f &v) { return Quatf(v.w, v.x, v.y, v.z); }

inline Vec4f quatToVec4(const Quatf &q) { return Vec4f(q.x, q.y, q.z, q.w); }

// ---------------------------------------------------------------------------
// Rotation4D: 4D rotation stored as a pair of unit quaternions (qL, qR).
// Action on v:  v' = qL * v * qR  (Rotation.md / RotateVector convention).
//
// qL is the left SU(2) factor. qR is the right SU(2) factor, stored directly
// from wedge/rates as exp(vR) with no extra conjugation. apply() right-multiplies
// by the stored qR exactly as written — same as Rotation.md ApplyRotation +
// RotateVector.
//
// Isoclinic vs double rotation (see Wikipedia: Rotations in 4-dimensional
// Euclidean space): A left-isoclinic rotation uses only qL (qR = identity);
// a right-isoclinic uses only qR (qL = identity). Every 4D rotation in SO(4)
// is uniquely (up to sign) the composition of one left- and one
// right-isoclinic, so arbitrary (qL, qR) is correct and covers all 4D
// rotations, including double rotations (two different angles in two invariant
// planes). Use fromLeftQuat/fromRightQuat when you want a single isoclinic
// rotation.
// ---------------------------------------------------------------------------

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

    void normalize()
    {
        qL.normalize();
        qR.normalize();
    }

    /** Simple rotation in one of the 6 coordinate planes: exactly one plane
     *  rotates, the orthogonal 2-plane stays fixed. axis1, axis2 in {0,1,2,3}
     *  for x,y,z,w; angle in radians.
     *
     *  Rule A — spatial planes (xy, xz, yz): qR = qL so the w-axis stays fixed.
     *  Rule B — planes involving w (xw, yw, zw): qR = conj(qL) so the two spatial
     *  axes not in the plane stay fixed. */
    static Rotation4D fromPlaneAngle(int axis1, int axis2, float angleRad)
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

    // --- Wedge-product / SU(2) construction (see quaternion4DRotation.md) ---

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
    static Rotation4D fromTwoPlanes(const Vec4f &u1, const Vec4f &v1, float angle1Rad,
                                    const Vec4f &u2, const Vec4f &v2, float angle2Rad)
    {
        Vec4f u1_normalized = u1.normalized();
        Vec4f v1_normalized = v1.normalized();
        Vec4f u2_normalized = u2.normalized();
        Vec4f v2_normalized = v2.normalized();
        float rXY = (u1_normalized.x * v1_normalized.y - u1_normalized.y * v1_normalized.x) * angle1Rad +
                    (u2_normalized.x * v2_normalized.y - u2_normalized.y * v2_normalized.x) * angle2Rad;
        float rYZ = (u1_normalized.y * v1_normalized.z - u1_normalized.z * v1_normalized.y) * angle1Rad +
                    (u2_normalized.y * v2_normalized.z - u2_normalized.z * v2_normalized.y) * angle2Rad;
        float rZX = (u1_normalized.z * v1_normalized.x - u1_normalized.x * v1_normalized.z) * angle1Rad +
                    (u2_normalized.z * v2_normalized.x - u2_normalized.x * v2_normalized.z) * angle2Rad;
        float rXW = (u1_normalized.x * v1_normalized.w - u1_normalized.w * v1_normalized.x) * angle1Rad +
                    (u2_normalized.x * v2_normalized.w - u2_normalized.w * v2_normalized.x) * angle2Rad;
        float rYW = (u1_normalized.y * v1_normalized.w - u1_normalized.w * v1_normalized.y) * angle1Rad +
                    (u2_normalized.y * v2_normalized.w - u2_normalized.w * v2_normalized.y) * angle2Rad;
        float rZW = (u1_normalized.z * v1_normalized.w - u1_normalized.w * v1_normalized.z) * angle1Rad +
                    (u2_normalized.z * v2_normalized.w - u2_normalized.w * v2_normalized.z) * angle2Rad;
        return fromRates(rXY, rYZ, rZX, rXW, rYW, rZW);
    }
 
     /** Continue building rotation from the 6 cardinal plane rates (wedge decomposition).
      *  vL = (R_yz+R_xw, R_zx+R_yw, R_xy+R_zw), vR = (-R_yz+R_xw, -R_zx+R_yw,
      * -R_xy+R_zw); then qL = exp(vL), qR = exp(vR). Same as Rotation.md
      * ApplyRotation. */
     static Rotation4D fromRates(float rXY, float rYZ, float rZX, float rXW, float rYW, float rZW)
     {
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
         const float eps = 1e-7f; // super small non-zero threshold for negligible axis
         Vec3f axis(vx, vy, vz);
 
         // if this is tiny, return identity
         float theta = axis.mag();
         if (theta < eps)
         {
             return Quatf::identity();
         }
 
         // otherwise, return the quaternion
         Quatf q;
         q.fromAxisAngle(theta, axis / theta);
         q.normalize();
         return q;
     }

    void prepend(const Rotation4D &newRot)
    {
        qL = newRot.qL.multiply(qL);
        qR = qR.multiply(newRot.qR);

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

// default face direction is FaceRight(1, 0, 0, 0), FaceUp(0, 1, 0, 0), FaceForward(0, 0, -1, 0), FaceAna(0, 0, 0, -1).
struct FaceDirection
{
    Vec4f faceRight{1.f, 0.f, 0.f, 0.f};
    Vec4f faceUp{0.f, 1.f, 0.f, 0.f};
    Vec4f faceForward{0.f, 0.f, -1.f, 0.f};
    Vec4f faceAna{0.f, 0.f, 0.f, -1.f};

    FaceDirection() = default;

    // only call when needed, don't call every frame
    void updateFaceDirection(const Rotation4D &rotation)
    {
        faceForward = rotation.apply(faceForward).normalized();
        faceRight = rotation.apply(faceRight).normalized();
        faceUp = rotation.apply(faceUp).normalized();
        faceAna = rotation.apply(faceAna).normalized();
    }

    // void faceForwardToward(const Vec4f &direction)
    // {
    //     // to faceRight to a certain direction,
    //     // calculate the Rotation4D quaternions set that would do that in the minimal steps
    //     // somehow return Rotation4D, fed back to object4D rotationState
    // }
};