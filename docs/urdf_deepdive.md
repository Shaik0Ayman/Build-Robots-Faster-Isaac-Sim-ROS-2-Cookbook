# URDF Deep Dive — Mass, Inertia, and Practical Tips

This document collects formulas, practices, and tips to compute and validate inertial parameters for URDF links so simulations in Isaac Sim, Gazebo, and other physics engines behave predictably.

**Inertia Basics**

- The inertia tensor (matrix) describes how mass is distributed relative to the center of mass (CoM). It must be symmetric and positive definite.
- Units: SI — mass in kilograms (kg), distances in meters (m), inertia in kg·m².

**Inertia Matrix (principal axes)**

When principal axes align with the link frame at the CoM, the inertia matrix is diagonal:

I = diag(Ixx, Iyy, Izz)

If the link reference frame is offset from the CoM, use the parallel axis theorem (see below).

**Closed-form inertia for common shapes**

Box (solid cuboid) w × h × d (x = width, y = height, z = depth):

$$I_{xx} = \frac{1}{12} m (y^{2} + z^{2})$$
$$I_{yy} = \frac{1}{12} m (x^{2} + z^{2})$$
$$I_{zz} = \frac{1}{12} m (x^{2} + y^{2})$$

Solid cylinder of mass m, radius r, height h (axis along cylinder axis):

- About central axial axis (along height):
  $$I_{axis} = \frac{1}{2} m r^{2}$$
- About any diameter axis (perpendicular to axis):
  $$I_{diameter} = \frac{1}{12} m (3r^{2} + h^{2})$$

Solid sphere of radius r:

$$I = \frac{2}{5} m r^{2}$$

Use the appropriate orientation when mapping these formulas into your URDF link frame.

**Parallel Axis Theorem (vector/matrix form)**

To shift an inertia tensor from the center of mass (C) to a parallel axis O separated by vector d (O = C + d):

$$I_O = I_C + m (\|d\|^{2} E - d d^{T})$$

where E is the 3×3 identity matrix and `d` is the vector from the CoM to the new origin.

For scalar/diagonal cases when d is along a single axis this simplifies to adding `m d^{2}` to the perpendicular inertia components.

**Practical Tips & Common Mistakes**

- Never leave inertia values at zero. Small but non-zero realistic values are required.
- Inertia must be consistent with mass and geometry. Scaling mass without updating inertia (or vice versa) breaks dynamics.
- Ensure the inertia tensor is positive definite (diagonal entries positive and matrix symmetric). Negative or non-physical values cause simulation instability.
- When using visual meshes for mass/inertia estimation, scale both mass and inertia when the mesh is scaled. For uniform scale `s`, inertia scales as `s^5` relative to original mass distribution — better to recompute analytically or via CAD.
- Use simplified collision geometry for dynamics and keep high-detail visual meshes only for rendering.

**Workflow Recommendations**

1. Author geometry in CAD and export mass/inertia properties directly when possible.
2. If CAD isn't available, decompose a link into primitive shapes (boxes, cylinders, spheres), compute mass/inertia for each, then combine them using parallel axis theorem.
3. Validate the URDF using tools such as `check_urdf` (from `urdf_parser_py`) and visualize CoM frames in RViz or your simulator to confirm link frames and axes alignment.

**Isaac Sim & Gazebo Notes**

- Isaac Sim reads URDF similarly to other simulators but expects correct joint axes and coherent inertial data. Misaligned joint axes cause articulation errors.
- Gazebo-specific `<gazebo>` tags and plugins are generally ignored by Isaac Sim — keep them if you dual-run but do not rely on them for Isaac-specific behavior.
- For high-speed dynamics (wheels, fast arms), prefer simplified collision meshes, tight inertia values from CAD, and smaller simulation time steps if needed.

**Quick checklist before simulation**

- [ ] Every link has `<inertial>` with realistic `mass` and `inertia` values.
- [ ] The inertia tensor is symmetric and positive definite.
- [ ] Joint `<axis>` values are defined in the joint frame and correctly oriented.
- [ ] Collision geometry is simplified and aligned with visual geometry where possible.
- [ ] For scaled meshes, recompute inertia rather than reusing previous values.

**References & Tools**

- CAD export tools (Fusion360, SolidWorks, Onshape) — compute MoI directly.
- `check_urdf` — basic URDF validation.
- `urdf_tutorial` and RViz visual checks for link frames and CoM.

If desired, example Python scripts to compute composite inertia from primitive shapes or a validation script can be provided upon request.
