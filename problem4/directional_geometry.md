# Directional Search And Localization Guarantees

The guarantees below use closed transmitting half-planes, reception radius at
least 1000 m, source positions in the closed 1800 m disk, and absolute bearing
error at most `ERR = 1.0050001` degrees. Optical clearance succeeds within 20 m
regardless of the transmitting direction. A source can be omnidirectional or
directional; its type and its transmitting normal are unknown to the planner.

## Discovery At 22 Stations

Let `R = 1999`, and let `u[k]` be the unit vector at angle `2*pi*k/7`.
The stations are the origin, the seven vertices `R*u[k]`, the seven radial
midpoints `(R/2)*u[k]`, and the seven side midpoints
`(R/2)*(u[k] + u[k+1])`. Indices wrap modulo seven. These are 22 distinct points.

The outer regular heptagon has inradius

```
1999*cos(pi/7) = 1801.0367669369357 > 1800.
```

Join its center to its vertices, then join the three side midpoints of every
resulting triangle. This partitions the heptagon into 28 triangles, with side
lengths `999.5`, `999.5`, and `1999*sin(pi/7)`, all strictly below 1000 m.

For any source `X`, select a mesh triangle containing it. Each vertex of this
triangle is within 999.5 m of `X`, since the triangle has diameter 999.5 m.
Write `X = sum(lambda[i]*V[i])`, with nonnegative weights summing to one.
For any transmitting normal `n`,

```
sum(lambda[i] * dot(n, V[i] - X)) = 0.
```

At least one vertex therefore satisfies `dot(n, V[i] - X) >= 0` and receives
the signal. This includes a source on an edge, a source at a vertex, and a
transmitting direction tangent to an edge. The statement of the problem
explicitly includes the 90-degree boundary of the transmitting semicircle.

Scanning all still-unknown channels at all 22 stations discovers every source.
Discovery can also stop after 16 distinct channels are found, since 16 is the
stated upper bound. The lower bound of 10 is not a sufficient stopping rule.

## A Contracting Pair Of G Probes

Suppose `A` has received a source with measured bearing axis `e`, perpendicular
unit vector `v`, and a certified distance upper bound `0 < U <= 1500`.
Let `epsilon = ERR*pi/180`. The true source has the form

```
X = A + r*(cos(delta)*e + sin(delta)*v),
0 <= r <= U,  abs(delta) <= epsilon.
```

Choose a transverse fraction `b`, and define

```
d = U/(2*cos(epsilon))
h = b*U
G+ = A + d*e + h*v
G- = A + d*e - h*v.
```

The implementation accepts a fraction only when `h > d*tan(epsilon)` and the
computed maximum source-to-probe distance is below 999.9 m. The fractions
`0.02`, `0.12`, and `0.25` satisfy both conditions for the stated error and
`U <= 1500`.

For either probe, the squared distance is maximized at `r = 0` or `r = U`,
and at an extreme allowed angle. Thus a common distance upper bound is

```
Q^2 = max(d*d + h*h,
          U*U + d*d + h*h - 2*U*(d*cos(epsilon) - h*sin(epsilon))).
```

With the above `d`, this becomes

```
Q/U = sqrt(1/(4*cos(epsilon)^2) + b*b + 2*b*sin(epsilon)).
```

At `b = 0.25`, `Q/U < 0.566875`, so `Q < 850.312` m.
At `b = 0.02`, `Q/U < 0.501178`, so `Q < 751.766` m.
Both probes are therefore within the minimum possible reception radius for
every source still consistent with the original observation.

### When A Probe Receives

Intersect the certified region with the new bearing wedge and the disk of
radius `Q` centered at the receiving probe. The next iteration can use that
probe as its received anchor with upper bound `Q` (or a tighter verified
bound from the region). This bound comes from geometry and does not assume an
omnidirectional source. A `near` response can be cleared immediately.

### When Both Probes Report No Signal

Since range cannot explain either miss, both probes are outside the source's
closed transmitting half-plane. Let `x = dot(X-A,e)`. Suppose that `x >= d`.
The segment from `A` to `X` then meets the line through `G+` and `G-` at

```
B = A + (d/x)*(X-A).
```

Its transverse coordinate has magnitude at most `d*tan(epsilon) < h`, so `B`
lies on `G+G-`. Since `A` received and the transmitting half-plane is convex
and has `X` on its boundary, the entire segment `AX`, including `B`, lies in
that closed half-plane. A convex combination of two strictly exterior points
cannot be in it. This is a contradiction, including the case `B = X`.

Consequently `x < d`, which gives the new distance bound

```
r < d/cos(epsilon) = U/(2*cos(epsilon)^2) < 0.500154*U.
```

The next iteration retains the old received anchor `A` and its bearing, clips
the region to `dot(X-A,e) <= d`, and uses this smaller upper bound. A pair of
misses must not create a fictitious bearing at either probe.

### Termination

Every completed iteration contracts the certified distance by at least the
factor `0.566875` when using one of the supported fractions no larger than
`0.25`, apart from the explicit one-micrometer numerical allowance. Starting
at 1500 m, at most eight completed iterations suffice for a bound below
19.9 m. At that point the anchor is a valid optical clearance point, whether
or not it remains on a transmitting boundary. A minimum enclosing circle
of radius at most 19.9 m can certify clearance sooner.

## Integration Requirements

- `bracketStep` requires an actual positive or bearing observation at its
  anchor, together with an independently certified distance upper bound.
- `afterReception` applies only the geometric distance disk. The caller must
  still process the received bearing with the normal observation machinery.
- `afterTwoMisses` applies only after both matching probes returned
  `no_signal`; never apply it after only one miss or after a failed request.
- A single `no_signal` cannot exclude a 1000 m disk around its station. It
  may be caused by transmitting direction. Such exclusions inherited from
  the omnidirectional heuristic must be removed.
- Transverse `F+` and `F-` points at the original anchor can be useful optional
  parallax probes. They have no unconditional range or reception guarantee.
- Use the source-to-probe vectors in the transmitting half-plane test.
  Bearings use the opposite, probe-to-source vectors.
- Clearance certification is geometric. Do not require an RF signal at the
  final optical clearance point.

The polygon clipping helpers deliberately approximate disks from outside.
The source remains in the resulting convex regions; geometric upper bounds
from their vertices are conservative. The header additionally expands
distance bounds and the double-miss clipping line by `1e-6` m.
