# 4D Nautilus — Architecture & Mathematics

This document defines the **mathematical model** of the 4D world, the **viewer frame**, **4D rotations**, and the two **dimension-reduction** paths (projection and slicing) that produce native 3D geometry for a standard graphics pipeline. It states **logic only**—no implementation algorithms.

Unless stated otherwise, coordinates are in **viewer-local** space after navigation.

---

## 1. Purpose

We explore a true **4D Euclidean** world while displaying on a **2D screen** through an unchanged **3D → 2D** graphics stack (Allolib / OpenGL).

| Stage | Dimension | Role |
|-------|-----------|------|
| World & navigation | 4D | Objects and viewer exist and move in **R⁴** |
| Reduction | 4D → 3D | Projection or slicing produces 3D meshes in viewer-local coordinates |
| Display | 3D → 2D | Standard lens, depth buffer, and rasterization |

**Design rule:** All 4D meaning is fixed **before** reduction. The 3D pipeline only **displays** the reduced scene; it does not reinterpret 4D orientation.

---

## 2. Coordinate conventions

World axes **(x, y, z, w)** match Allolib / OpenGL for the spatial part:

| Axis | Direction | Code |
|------|-----------|------|
| **+x** | Right | `Vec4f` component 0 |
| **+y** | Up | `Vec4f` component 1 |
| **+z** | Backward (into the screen) | `Vec4f` component 2 |
| **+w** | Kata (toward the viewer / “in front” in 4D) | `Vec4f` component 3 |

A **4D point** is $p = (x, y, z, w)$ , `Vec4f position = (x, y, z, w)`.


---

## 3. Mathematical objects and code correspondence

### 3.1 Rigid 4D objects — `Object4D`

An object in 4D has:

- **World position P** in **R⁴** `Object4D::pos`
- **Orientation R** in **SO(4)** `Object4D::rotationState` (`Rotation4D`)
- **Orthonormal frame** `Object4D::faceDirection` (`FaceDirection`)

Subclasses (e.g. hypervolume nautilus) store geometry in **object-local** 4D coordinates; world points are obtained by applying **R** and translation.

### 3.2 4D viewer — `Nav4D`

The **4D camera** is a `Nav4D` instance (extends `Object4D`). It supplies:

- Observer position **P**
- Orientation **R**
- Viewer basis vectors (Section 4)

Navigation updates **P** and **R** **before** any projection or slicing.

### 3.3 Viewer basis — `FaceDirection`

Let **R** act on column vectors. Default world-aligned basis vectors are stored in `FaceDirection`; after rotation:

$$
\hat{R} = R \cdot \hat{e}_x \\
\hat{U} = R \cdot \hat{e}_y \\
\hat{F}_{\text{stored}} = R \cdot \hat{e}_z' \\
\hat{N}_{\text{stored}} = R \cdot \hat{e}_w'
$$

Defaults: **ê_x** = (1,0,0,0), **ê_y** = (0,1,0,0), **ê_z'** = (0,0,−1,0), **ê_w'** = (0,0,0,−1).

| Symbol | Meaning | Code field |
|--------|---------|------------|
| **R̂** | Local right (**+x**) | `faceDirection.faceRight` |
| **Û** | Local up (**+y**) | `faceDirection.faceUp` |
| **F̂** | Local forward (OpenGL **−z** viewing direction) | minus `faceDirection.faceForward` |
| **N̂** | Hyperplane normal (**+w** / kata side) | minus `faceDirection.faceAna` |

The negations align the stored frame with OpenGL (forward = negative **z**) and kata = positive **w**.

The frame is **orthonormal** and **right-handed** in **R⁴** when derived from a proper **R** in **SO(4)**.

The default viewer facing direction is 
```c++
std::array<Vec4f, 4> face =
        {
            Vec4f(1.f, 0.f, 0.f, 0.f), // right
            Vec4f(0.f, 1.f, 0.f, 0.f), // up
            Vec4f(0.f, 0.f, -1.f, 0.f), // forward (OpenGL -z)
            Vec4f(0.f, 0.f, 0.f, -1.f) // ana (negative w, arbitrary mimicing OpenGL standard)
        };
```

### 3.4 Reduction modules

| Module | Role |
|--------|------|
| `FProjecter` | Perspective reduction along **N̂** (**w** depth cue) → see-through 3D meshes |
| `FSlicer` | Hyperplane cross-section → solid 3D meshes |

Output of both reducers: **3D vertex coordinates** in viewer-local **(x_3D, y_3D, z_3D)** suitable for `al::Mesh` and Allolib `Graphics`.

---

## 4. World space and viewer-local space

### 4.1 World coordinates

Geometry and object poses live in **world** 4D coordinates.

### 4.2 Viewer-local coordinates

For viewer position **P** and orientation **R**, the **viewer-local** point is:

$$
p_{\text{local}} = R^{-1} \cdot (p_{\text{world}} - P)
$$

Code concept: `toViewerLocal(viewer, worldPoint)` in `Manifolds4D.hpp`, using `Rotation4D::inverse()` and `Rotation4D::apply()`.

All visibility, projection, and slicing tests use **p_local** unless explicitly stated.

### 4.3 Dropping to display 3D

For a point on the slice hyperplane (or after projection), display coordinates use the viewer’s 3D subspace:

$$
x_{3D} = p_{\text{local}} \cdot \hat{R} \\
y_{3D} = p_{\text{local}} \cdot \hat{U} \\
z_{3D} = p_{\text{local}} \cdot \hat{F}
$$

When **p_local** lies on the hyperplane **H** (Section 6), **p_local · N̂ = 0** and the fourth coordinate is redundant.

---

## 5. Rotations in R⁴

This section restates the rotation mathematics (formerly `Rotation.md`) and maps symbols to `Rotation4D` in `FMath.hpp`.

### 5.1 Group structure

Proper 4D rotations form **SO(4)**, with:

$$
SO(4) \cong (SU(2) \times SU(2)) / \{\pm 1\}
$$

Every **R** in **SO(4)** is represented (up to sign) by a pair of **unit quaternions** **q_L**, **q_R** on **S³**.

| Math | Code |
|------|------|
| **q_L** | `Rotation4D` left factor (`getQL()`) |
| **q_R** | `Rotation4D` right factor (`getQR()`) |

The imaginary units **i**, **j**, **k** in **q_L** and **q_R** are **abstract** **SU(2)** generators; they are not the physical **x**, **y**, **z** axes. The six physical rotation degrees of freedom appear when **q_L** and **q_R** interact.

### 5.2 Vectors as quaternions

Identify **v** = (x, y, z, w) with the quaternion **V** = w + xi + yj + zk. Code: `vec4ToQuat` / `quatToVec4` (scalar part = **w**).

### 5.3 Action on vectors

A rotation acts by **sandwiching**:

$$
v' = q_L \cdot V \cdot q_R
$$

Code: `Rotation4D::apply()`.

### 5.4 Accumulating rotations

If **v₁** = **q_L1** · **V₀** · **q_R1** and a second rotation **(q_L2, q_R2)** is applied:

$$
v_2 = q_{L2} \cdot q_{L1} \cdot V_0 \cdot q_{R1} \cdot q_{R2}
$$

Accumulated state:

$$
q_L' = q_{L2} \cdot q_{L1}, \qquad q_R' = q_{R1} \cdot q_{R2}
$$

| Operation | Math | Code |
|-----------|------|------|
| Apply new rotation after current | **q_L'** = **q_L,new** · **q_L**, **q_R'** = **q_R** · **q_R,new** | `Rotation4D::append()`, `Object4D::appendRotation()` |

Left factors multiply on the **left**; right factors on the **right**.

### 5.5 Inverse

For unit quaternions, **q⁻¹** = conjugate **q̄**. Undoing rotation:

$$
v_0 = \bar{q}_L' \cdot V' \cdot \bar{q}_R'
$$

Code: `Rotation4D::inverse()`.

### 5.6 Cardinal plane rates (wedge decomposition)

A rotation in the plane spanned by orthonormal **u**, **v** with angle **θ** decomposes into six **cardinal bivector** rates:

$$
\begin{aligned}
R_{xy} &= (u_x v_y - u_y v_x)\,\theta \\
R_{yz} &= (u_y v_z - u_z v_y)\,\theta \\
R_{zx} &= (u_z v_x - u_x v_z)\,\theta \\
R_{xw} &= (u_x v_w - u_w v_x)\,\theta \\
R_{yw} &= (u_y v_w - u_w v_y)\,\theta \\
R_{zw} &= (u_z v_w - u_w v_z)\,\theta
\end{aligned}
$$

**Double rotation:** For two orthogonal planes, compute rates for each plane and **add** corresponding **R_ab** before building **(q_L, q_R)**.

Code: `Rotation4D::fromPlane()`, `Rotation4D::fromTwoPlanes()`, `Rotation4D::fromPlaneAngle(axis1, axis2, angle)`.

### 5.7 Building q_L and q_R from rates

Generator vectors in **su(2)**:

$$
\begin{aligned}
v_L &= (R_{yz} + R_{xw})\,\mathbf{i} + (R_{zx} + R_{yw})\,\mathbf{j} + (R_{xy} + R_{zw})\,\mathbf{k} \\
v_R &= (-R_{yz} + R_{xw})\,\mathbf{i} + (-R_{zx} + R_{yw})\,\mathbf{j} + (-R_{xy} + R_{zw})\,\mathbf{k}
\end{aligned}
$$

Half-angles **θ_L** = |**v_L**|, **θ_R** = |**v_R**|. Exponential map:

$$
q_L = \cos(\theta_L) + \sin(\theta_L)\,\frac{v_L}{\theta_L}, \qquad
q_R = \cos(\theta_R) + \sin(\theta_R)\,\frac{v_R}{\theta_R}
$$

(at identity if |**v**| ≈ 0).

Code: `Rotation4D::fromRates()`, `Rotation4D::quatFromImaginary()`.

### 5.8 Interpolation

For states **(q_L,A, q_R,A)** and **(q_L,B, q_R,B)** and **t** in [0, 1]:

$$
q_L(t) = \mathrm{Slerp}(q_{L,A}, q_{L,B}, t), \qquad
q_R(t) = \mathrm{Slerp}(q_{R,A}, q_{R,B}, t)
$$

with the usual shortest-path quaternion Slerp (negate **q_B** if **q_A · q_B < 0**).

Code: `Rotation4D::slerp()`.

### 5.9 Face frame from rotation

Canonical basis vectors **ê_i** rotated by **R** give the viewer/object frame (Section 3.3). Code: `FaceDirection::updateFaceDirection(rotation)` or lazy `Rotation4D::apply()` on defaults.

---

## 6. Slicing reduction (4D → 3D)

**Idea:** A 3D **hyperplane** cuts 4D space; the cut is a 3D volume rendered as ordinary mesh geometry.

### 6.1 Hyperplane

Observer at **C** (origin in viewer-local: **C** = **0**). Hyperplane **H** with unit normal **N̂**:

$$
H = \{ P : \hat{N} \cdot (P - C) = 0 \}
$$

In viewer-local coordinates with **N̂** aligned to the local **w** axis, this is the plane **w = w_plane** for some offset **w_plane**. Code setting: `HyperSliceSettings::wPlane`.

### 6.2 Signed distance

For vertex **V**:

$$
d(V) = \hat{N} \cdot (V - C)
$$

| Sign of **d** | Interpretation |
|---------------|----------------|
| **d > 0** | Kata side of **H** |
| **d < 0** | Ana side of **H** |
| **d = 0** | On **H** |

### 6.3 Intersecting 4D edges

For edge **A**–**B**, if **d_A** and **d_B** have opposite signs:

$$
t = \frac{d_A}{d_A - d_B}, \qquad P_{\text{int}} = A + t\,(B - A)
$$

### 6.4 Boundary of a 4D cell

A 4D hypersurface is tiled by **3-simplices** (tetrahedra) or higher cells whose facets are such simplices. Slicing a 4-simplex yields a 3D polyhedron; triangulating its boundary produces **3D triangles** for the display mesh.

Map **P_int** to **(x_3D, y_3D, z_3D)** via Section 4.3.

Code: `FSlicer.hpp`, `slice4SimplicesViewerLocal()`.

---

## 7. Projection reduction (4D → 3D)

**Idea:** Use **perspective along N̂** (the **w** direction in viewer-local space) as a depth cue, analogous to **z** perspective in 3D.

### 7.1 Visibility (intended policy)

In viewer-local coordinates, only points with **w_local** at or “behind” the observer’s parallel 3-space are visible—same logical rule as “see only **z ≤ z_camera**” in 3D:

- Same **w** as the observer’s reference 3-space, or
- Smaller **w** (farther in the ana direction, depending on sign convention)

(Exact culling policy is a projecter setting in `FProjecter`.)

### 7.2 Perspective scale

Given local coordinates **p_local** = (x, y, z, w), a **perspective law** **s(w)** maps off-parallel 3-spaces into the observer’s reference 3-space with foreshortening:

$$
(x_{3D}, y_{3D}, z_{3D}) = s(w)\,(x, y, z)
$$

Examples to support (not fixed to one law):

- One-point perspective in **w**
- Multi-point or curvilinear variants

Current code uses a specific **s(w)** piecewise law in `projectLocal4Dto3D()` (`Manifolds4D.hpp`); `FProjecter` will generalize this.

### 7.3 Seen-through geometry

Projection keeps **connectivity** across **w**: the result is a **transparent** 3D mesh (lines or surfaces with alpha), not a solid slice.

Code today: `drawProjectedShadow()`, `Nautilus4D::drawProjected()`.

---

## 8. Three-layer scene model (World 1 / 2 / 3)

Unlike Allolib’s standard 3D path (world `Vec3f` + GPU **view matrix** → pixels), this project’s **deliverable is 3D mesh geometry** (`al::Mesh`) that Allolib draws with a **fixed** display camera. Reduction is a **CPU-side geometry pass** (GPU later optional).

Three **derived layers** sit in the application (conceptually in `FApp`; not all are separate stored buffers yet):

```text
┌─────────────────────────────────────────────────────────────────┐
│  World 1 — Navigable 4D world (authoritative)                   │
│  • Geometry generators (Nautilus4D, hypervolume, …)             │
│  • Each Object4D: pos, rotationState, object-local verts        │
│  • World 4D vertex pools + 4D topology (edges, simplices, …)    │
│  • Nav4D camera pose (also an Object4D)                         │
└────────────────────────────┬────────────────────────────────────┘
                             │ Nav4D localize (sync)
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  World 2 — Viewer-local 4D (cache / playground for reducers)  │
│  • Vec4f positions: p_local = R⁻¹(p_world − C) per vertex       │
│  • Fixed orthonormal coords (x,y,z,w scalars), NOT faceDirection│
│  • Topology: references World 1 (see §8.2)                      │
└────────────────────────────┬────────────────────────────────────┘
                             │ ProjectorPlane or Slicer4D
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  World 3 — Display 3D (Allolib-ready)                           │
│  • al::Mesh (Vec3f vertices + connectivity baked in)            │
│  • Fixed al::Nav at origin / −z for screen framing only         │
└─────────────────────────────────────────────────────────────────┘
```

**Roles:**

| Layer | Owns | Does not own |
|-------|------|----------------|
| **Nav4D** | Camera pose; **localize** World 1 → World 2 | Projection, slicing, `al::Mesh` |
| **Projector / Slicer** | World 2 → World 3 reduction | Object poses, generator parameters |
| **Generators** | World 1 geometry + 4D topology | Viewer frame, 3D output |

**Invalidation (rebuild World 2 when):** Nav4D moves; any object pose changes; generator parameters change.

**Invalidation (rebuild World 3 when):** World 2 changes; reducer settings change (projection mode, `wPlane`, etc.).

### 8.1 Projection vs slicing in World 3

| | **Projection** | **Slicing** |
|---|----------------|-------------|
| **Input** | World 2 positions + World 1 **edge** lists | World 2 positions + World 1 **cell** lists (4-simplices, …) |
| **Output positions** | One `Vec3f` per surviving 4D vertex (after cull) | **New** intersection points; **w** dropped → $(x,y,z)$ |
| **Output topology** | **Same** edge graph as World 1 (drop culled edges) | **New** mesh; cross-section triangulation |
| **World 1 on output** | Connectivity **referenced** when building mesh | Connectivity is **input only**, not carried over |

Slicing **never** “fetches connectivity from World 1” on output — the slice mesh is a generated artifact (`Slicer4D::Result`).

### 8.2 World 2 topology references (one vs many generators)

World 2 stores **localized positions only**. Connectivity stays defined in **World 1** per generator, referenced until World 3 is built.

**Single generator (current):** One vertex array + one edge/simplex list; indices are local to that object. Trivial.

**Multiple generators — two viable patterns:**

**A. Per-object chunks (recommended first)**

Each placed object exports a **geometry source** in World 1:

```text
GeometrySource {
  objectId
  worldVerts[]           // or rebuild from object-local + pose
  edges[] / cells4[]     // indices into THIS object's vertex array
}
```

World 2 holds a **parallel list**:

```text
LocalizedChunk {
  positions[]            // same length as source.worldVerts
  const GeometrySource* source   // non-owning ref into World 1
}
```

Nav4D localizes **per chunk**: `positions[i] = toLocal(source.worldVert[i])`. Reducers iterate chunks: for projection, walk each source’s edges; for slicing, feed each source’s cells (or merge first — see B).

**B. Unified scene pool (scale-up)**

`buildWorldScene()` concatenates all objects into one vertex array; each edge/simplex stores **global indices** (with index offsets per object). World 2 is one big `positions[]` aligned 1:1. One pointer pair to global edge/cell lists.

Use **A** for clarity and per-object dirty flags; use **B** when draw/reducer loops must be single-pass and vertex counts are large.

**Rule:** Indices in World 1 topology always mean “vertex $k$ within **this** object’s pool” (local indices) **or** “vertex $k$ in the global pool” (global indices) — pick one convention and stick to it. World 2 never renumbers; it only transforms positions in place or in a parallel array with the **same indexing**.

### 8.3 World 3 and `al::Mesh`

**Yes — World 3 should be `al::Mesh` (or a small wrapper) for both reducers.**

Allolib does not draw “positions + external connectivity.” `al::Mesh` **is** vertices + optional indices + primitive mode (`LINES`, `TRIANGLES`, `POINTS`).

**Projection:** You do **not** keep World 3 as “Vec3f array + pointer to World 1 edges” at rest. Each sync frame:

1. Read World 2 `Vec4f` + World 1 edge list.
2. Project each endpoint → `Vec3f`.
3. **Emit** line/point pairs into `al::Mesh` (skip culled edges).

Connectivity is **baked into** the mesh (line list or index buffer). World 1 edges are the **recipe** during build, not a runtime reference on World 3. Same as today’s `drawNautilusProjection` loop.

**Slicing:** Already produces standalone `al::Mesh` sets (`volume`, `surface`, `curves`, `points`) with new vertices and new connectivity inside each mesh.

Optional wrapper:

```text
DisplayScene {
  enum Mode { Projection, Slice };
  Mode mode;
  al::Mesh projectionLines;   // or points
  Slicer4D::Result sliceMeshes;
}
```

**CPU first** for localize + reduce; GPU compute is an optional later optimization (localize/project parallelize easily; slicing is harder).

---

## 9. End-to-end pipeline (summary)

```text
World 1: generators + Nav4D  (4D world simulation)
    → localize →  World 2: viewer-local Vec4f  (fixed x,y,z,w frame)
    → reduce   →  World 3: al::Mesh  (display 3D)
    → Allolib Graphics + fixed al::Nav  →  2D screen
```

| Step | Math | Primary code (current / planned) |
|------|------|----------------------------------|
| Object in world | **p_world** = **C_obj** + **R_obj** · **p_obj** | `Object4D`, `Nautilus4D`, hypervolume |
| Navigate | Update viewer **C**, **R** | `Nav4D`, `Object4D::step()` |
| Localize (World 2) | **p_local** = **R⁻¹** · (**p_world** − **C**) | `Nav4D::toLocal()` |
| Project (World 3) | **s(w)** scaling, cull, → `Vec3f`; bake edges into `al::Mesh` | `ProjectorPlane` |
| Slice (World 3) | **w = w_plane**, intersect, triangulate → `al::Mesh` | `Slicer4D` |
| Display | Standard 3D draw on reduced meshes | `FApp`, `al::Graphics` |

The **dimensional boundary** for 4D meaning is World 2. Allolib’s 3D `Nav` applies **after** World 3 exists, for screen framing only—not for 4D exploration.

---

## 10. Related files

| Topic | File |
|-------|------|
| Vectors, `Rotation4D`, `FaceDirection` | `FMath.hpp` |
| Object pose | `Object4D.hpp` |
| 4D viewer | `Nav4D.hpp` |
| World → local, projection helpers | `Manifolds4D.hpp` |
| Slicing | `FSlicer.hpp` |
| Projection | `FProjection.hpp`, `FProjectorPlane.hpp` |
| Application / scene layers (planned explicit) | `FApp.cpp` |
| Rotation reference (legacy prose) | `Rotation.md` |

---

## 11. Glossary

| Term | Meaning |
|------|---------|
| **Kata** | Positive **w**; toward the viewer in 4D |
| **Ana** | Negative **w**; away from the viewer |
| **Viewer-local** | Coordinates in the Nav4D frame after **R⁻¹ · (p_world − C)** |
| **Reduction** | Projection or slicing: 4D → 3D display geometry |
| **World 1 / 2 / 3** | Navigable 4D world; viewer-local 4D cache; display `al::Mesh` |
| **GeometrySource** | World-1 object: world verts + 4D topology (edges/cells) |
| **LocalizedChunk** | World-2: viewer-local positions + non-owning ref to one GeometrySource |
| **Parallel 3-space** | Fixed-**w** affine subspace; “copy of 3D space” at one **w** |
