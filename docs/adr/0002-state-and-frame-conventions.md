# ADR-0002: State vector, frames and attitude conventions

- **Status:** accepted
- **Date:** 2026-08-18
- **Deciders:** project authors

## Context

Roughly half the defects in flight software are convention mismatches. A
quaternion that one module treats as scalar-first and another as scalar-last, a
rotation matrix that is transposed relative to what its caller assumes, an
angle-of-attack sign that flips between the aerodynamic model and the
linearisation — none of these produce a crash. They produce plausible numbers
that are wrong, and they survive review because every individual file is
internally consistent.

The only defence is to write the conventions down once, in full, with the
matrices spelled out, and to make every header point at that one place.

## Decision

The conventions below are normative for the whole of galata. Every source file
that touches a frame, an attitude or a state cites this ADR.

### Frames

**Navigation frame — NED.** x north, y east, z **down**. Flat-Earth and
non-rotating by default. A round-Earth WGS-84 option exists behind a flag; when
the flat-Earth assumption is in force it is recorded in the output rather than
left implicit.

**Body frame — FRD.** x forward out the nose, y out the **right** wing,
z **down** through the belly. The origin is a documented geometric reference
point fixed to the airframe, *not* the centre of gravity. The CG is carried as
an explicit offset from that reference point so that CG position is a parameter
that can be swept, which is the entire point of a CG sweep study.

> This fixes where geometry and aerodynamic data are referenced. It does not by
> itself say which point the *equations of motion* are written about, and this
> record originally left that ambiguous.
> [ADR-0006](0006-equations-of-motion-about-the-cg.md) resolves it: the
> dynamics are written about the centre of gravity, and the CG offset appears
> only in the transfer of the aerodynamic wrench from the reference point.

**Stability frame.** The body frame rotated by **−α** about the body y-axis, so
that its x-axis lies along the projection of the velocity vector into the body
xz-plane.

**Wind frame.** The stability frame rotated by **+β** about the stability
z-axis, so that its x-axis lies along the velocity vector.

The rotation chain, with `R_b<-a` denoting the matrix that transforms *vector
components* from frame `a` to frame `b`:

```
R_wind<-body = Rz(beta) * Ry(-alpha)

         [  cos b  sin b  0 ]        [ cos a  0  sin a ]
Rz(b) =  [ -sin b  cos b  0 ]  Ry(-a) = [   0    1    0   ]
         [    0      0    1 ]        [ -sin a 0  cos a ]

               [  cos b cos a   sin b   cos b sin a ]
R_wind<-body = [ -sin b cos a   cos b  -sin b sin a ]
               [   -sin a         0        cos a    ]
```

This composition is fixed by requiring that `R_wind<-body` map the body-axis
velocity components onto `[V, 0, 0]`.

### Attitude

**Unit quaternion, Hamilton convention, scalar-first, `q = [w, x, y, z]`,
representing the body-to-NED rotation.** That is: `q` rotates a vector's
components *from* the body frame *to* the navigation frame,

```
v_ned = q (x) v_body (x) q*
```

where `(x)` is the Hamilton product. Written out, `R_ned<-body` is

```
        [ 1-2(y^2+z^2)   2(xy - wz)    2(xz + wy)  ]
R  =    [  2(xy + wz)   1-2(x^2+z^2)   2(yz - wx)  ]
        [  2(xz - wy)    2(yz + wx)   1-2(x^2+y^2) ]
```

and `R_body<-ned = R_ned<-body^T`.

The kinematic equation, with body-axis angular rates `w_b = [p, q, r]`:

```
qdot = 0.5 * q (x) [0, p, q, r]

     = 0.5 * [ -x p - y q - z r,
                w p + y r - z q,
                w q + z p - x r,
                w r + x q - y p ]
```

**Euler angles are output, never state.** The 3-2-1 (ZYX) sequence — yaw `psi`
about z, then pitch `theta` about y, then roll `phi` about x — defines
`R_body<-ned = Rx(phi) Ry(theta) Rz(psi)`, giving

```
              [    cos th cos ps                cos th sin ps            -sin th   ]
R_body<-ned = [ sin ph sin th cos ps         sin ph sin th sin ps      sin ph cos th]
              [   - cos ph sin ps              + cos ph cos ps                     ]
              [ cos ph sin th cos ps         cos ph sin th sin ps      cos ph cos th]
              [   + sin ph sin ps              - sin ph cos ps                     ]
```

and the extraction

```
theta = asin( 2 (w y - x z) )
phi   = atan2( R_body<-ned[1][2], R_body<-ned[2][2] )
psi   = atan2( R_body<-ned[0][1], R_body<-ned[0][0] )
```

Because Euler angles are derived and never integrated, gimbal lock is not a
singularity of the dynamics. It remains a singularity of the *reporting*: at
`theta = ±90°` the `phi`/`psi` extraction is ill-conditioned, and the reporting
layer flags that rather than pretending otherwise.

**Renormalisation.** After every integrator step the quaternion is renormalised
by direct division, `q <- q / ||q||`. For fixed-step RK4 the per-step norm error
is small and the correction is a deterministic, branch-free operation, which is
what the determinism policy needs — an "only renormalise when the drift exceeds
a threshold" scheme introduces a data-dependent branch and with it a source of
platform-dependent divergence.

**The double cover is not collapsed during integration.** `q` and `-q` represent
the same rotation, and it is tempting to canonicalise by forcing `w >= 0`. That
is wrong here: a manoeuvre that carries `w` through zero would see the state
vector jump discontinuously, which destroys any finite-difference Jacobian taken
across that point. Canonicalisation is applied only when quaternions are
compared or serialised for a regression fixture, never inside the state.

### State vector

```
x = [ p_n  p_e  p_d      position,       NED,         m
      u    v    w        velocity,       body axes,   m/s
      q_w  q_x  q_y  q_z attitude,       body -> NED, dimensionless
      p    q    r ]      angular rate,   body axes,   rad/s
```

Thirteen components, in that order. The order is part of the contract: it is the
row and column order of every A and B matrix galata produces, and a linearisation
whose rows are permuted relative to its documentation is worthless.

Derived quantities, defined here once and used everywhere:

```
V     = || (u, v, w) ||           airspeed,          m/s
alpha = atan2(w, u)               angle of attack,   rad
beta  = asin(v / V)               sideslip angle,    rad
```

with the inverse `(u, v, w) = V (cos a cos b, sin b, sin a cos b)`. `atan2` is
used for `alpha` rather than `atan(w/u)` so that the definition remains correct
for rearward flight rather than silently folding it into the forward half-plane.
Both directions of this transformation are tested against each other.

A wind-axis formulation `(V, alpha, beta)` replaces `(u, v, w)` for trim and
linearisation, because trim conditions are naturally stated in those variables.
The transformation is tested in both directions rather than assumed.

### Equations of motion

Full nonlinear six-degree-of-freedom rigid body with a **general inertia
tensor**. `I_xz` is not assumed zero: it is not zero for real aircraft, and the
assumption is the difference between a correct Dutch roll and a plausible one.

```
m (udot + q w - r v) = X
m (vdot + r u - p w) = Y
m (wdot + p v - q u) = Z

[I] {pdot qdot rdot}^T + {p q r}^T x ([I] {p q r}^T) = {L M N}^T
```

Gravity is expressed in NED and rotated into the body frame:
`g_body = R_body<-ned [0, 0, g]^T`.

## Alternatives considered

**Scalar-last quaternions `[x, y, z, w]`.** Eigen's `Quaterniond` stores
coefficients in scalar-last order internally, and several robotics stacks use
scalar-last throughout, so this would remove one reordering at the Eigen
boundary. Rejected: the flight-dynamics literature and the reference documents
this project validates against are overwhelmingly scalar-first, and a reader
checking a sign against a textbook is the person this convention exists to
serve. Eigen's constructor is explicitly `Quaterniond(w, x, y, z)`, so the
boundary is a constructor call rather than a shuffle.

**JPL (scalar-last, left-handed composition) quaternions.** Used in parts of the
spacecraft-navigation world. Rejected for the same reason, and because mixing
Hamilton and JPL conventions in one codebase is the single most productive
source of attitude bugs in the field.

**Euler angles as integrated state.** Simpler to read in a debugger and removes
the normalisation step entirely. Rejected: the 3-2-1 sequence is singular at
`theta = ±90°`, and a tool whose stated purpose includes post-stall and
high-attitude simulation cannot carry a singularity in its state vector.

**ENU navigation frame and FLU body frame.** The ROS convention. Rejected: NED
and FRD are what the aerospace literature, MIL specifications and flight-control
practice use, and this tool's users read those documents.

**Origin of the body frame at the CG.** Simplifies the equations of motion by
deleting the CG-offset terms. Rejected outright: it makes CG position
unrepresentable, and "sweep CG from 25% to 40% MAC and show me where the short
period leaves Level 1" is a workflow this tool exists to serve.

## Consequences

- Every public struct field carries its unit in a comment; see
  [ADR-0003](0003-strict-si-and-boundary-conversion.md).
- The state ordering above is a compatibility surface. Changing it changes the
  meaning of every exported A, B, C, D matrix and is a major-version event.
- Property-based tests are mandatory for this layer: round-trip
  quaternion/Euler/DCM conversions, composition associativity, normalisation
  invariance, and the `(u,v,w) <-> (V,alpha,beta)` round trip. These are cheap
  to write and they are the tests that catch a transposed matrix.
- Interoperating with a scalar-last library requires an explicit conversion at
  the boundary. That conversion lives in one place and is tested.

## Revisit when

Never, for the 1.x series. These are compatibility surfaces, not preferences.
