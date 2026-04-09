#!/usr/bin/env python3
"""Propeller design and performance estimation tool for UAV testing.

This script estimates:
- Chord and twist distributions along the blade
- Sectional aerodynamic state (AoA, Reynolds, local thrust/torque)
- Total thrust, torque, and power
- Non-dimensional coefficients (CT, CP, CQ)
- Figure of merit and ideal induced velocity power check

Model notes
-----------
The implementation uses a practical Blade Element + Momentum style approach with:
- Annular momentum induced velocity initialization
- Geometric pitch-driven twist profile
- Lift/drag from a simple parameterized airfoil model

This is intended for preliminary design and bench-test planning, not final certification.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, asdict
from typing import List, Dict, Any


@dataclass
class PropellerInputs:
    diameter_m: float
    hub_radius_m: float
    blades: int
    rpm: float
    flight_speed_mps: float
    rho: float
    mu: float
    pitch_at_75_m: float
    tip_chord_m: float
    root_chord_m: float
    radial_sections: int
    cl_alpha_per_rad: float
    cl_max: float
    cd0: float
    k_induced_drag: float


@dataclass
class SectionResult:
    r_m: float
    chord_m: float
    beta_deg: float
    phi_deg: float
    alpha_deg: float
    reynolds: float
    cl: float
    cd: float
    dT_N: float
    dQ_Nm: float


@dataclass
class PropellerResults:
    inputs: Dict[str, Any]
    blade_radius_m: float
    area_disk_m2: float
    n_rev_per_sec: float
    sections: List[SectionResult]
    thrust_N: float
    torque_Nm: float
    power_W: float
    ct: float
    cp: float
    cq: float
    figure_of_merit: float
    ideal_induced_velocity_mps: float
    ideal_power_W: float


def lerp(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
    if x1 == x0:
        return y0
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0)


def build_chord_distribution(inp: PropellerInputs, r: float, R: float) -> float:
    return lerp(r, inp.hub_radius_m, R, inp.root_chord_m, inp.tip_chord_m)


def build_twist_distribution(inp: PropellerInputs, r: float, R: float) -> float:
    # Use pitch at 75% radius as reference: tan(beta_75) = pitch / (2*pi*r_75)
    r_75 = 0.75 * R
    beta_75 = math.atan2(inp.pitch_at_75_m, 2.0 * math.pi * r_75)

    # Keep nearly constant geometric pitch => beta(r) = atan(P/(2*pi*r))
    beta_r = math.atan2(inp.pitch_at_75_m, 2.0 * math.pi * r)

    # Slightly flatten near tip for manufacturability (blend 10% toward beta_75)
    flatten_weight = max(0.0, min(1.0, (r - 0.85 * R) / (0.15 * R)))
    beta_r = (1.0 - 0.1 * flatten_weight) * beta_r + 0.1 * flatten_weight * beta_75
    return beta_r


def section_aero_coefficients(alpha_rad: float, inp: PropellerInputs) -> tuple[float, float]:
    cl = inp.cl_alpha_per_rad * alpha_rad
    cl = max(-inp.cl_max, min(inp.cl_max, cl))
    cd = inp.cd0 + inp.k_induced_drag * cl * cl
    return cl, cd


def estimate_induced_velocity(thrust_guess: float, rho: float, area: float, v_inf: float) -> float:
    # For axial flow through actuator disk in forward speed: T = 2*rho*A*vi*(V_inf + vi)
    # Solve quadratic: 2*rho*A*vi^2 + 2*rho*A*V_inf*vi - T = 0
    a = 2.0 * rho * area
    b = 2.0 * rho * area * v_inf
    c = -thrust_guess

    disc = b * b - 4.0 * a * c
    if disc < 0:
        return 0.0
    vi = (-b + math.sqrt(disc)) / (2.0 * a)
    return max(0.0, vi)


def solve_propeller(inp: PropellerInputs, iterations: int = 25) -> PropellerResults:
    R = inp.diameter_m / 2.0
    A = math.pi * R * R
    n = inp.rpm / 60.0
    omega = 2.0 * math.pi * n

    # initialize induced velocity from a mild static thrust estimate
    thrust_est = 0.5 * inp.rho * A * (0.6 * omega * R) ** 2 * 0.08
    vi = estimate_induced_velocity(thrust_est, inp.rho, A, inp.flight_speed_mps)

    section_data: List[SectionResult] = []

    for _ in range(iterations):
        section_data = []
        thrust = 0.0
        torque = 0.0

        dr = (R - inp.hub_radius_m) / inp.radial_sections

        for i in range(inp.radial_sections):
            r = inp.hub_radius_m + (i + 0.5) * dr
            chord = build_chord_distribution(inp, r, R)
            beta = build_twist_distribution(inp, r, R)

            v_axial = inp.flight_speed_mps + vi
            v_tan = omega * r
            v_rel = math.hypot(v_axial, v_tan)
            phi = math.atan2(v_axial, v_tan)
            alpha = beta - phi

            cl, cd = section_aero_coefficients(alpha, inp)

            q = 0.5 * inp.rho * v_rel * v_rel
            dL = q * chord * cl * dr
            dD = q * chord * cd * dr

            # Resolve to thrust/torque for one blade element, then multiply by blade count
            dT_one = dL * math.cos(phi) - dD * math.sin(phi)
            dFt_one = dL * math.sin(phi) + dD * math.cos(phi)
            dQ_one = dFt_one * r

            dT = dT_one * inp.blades
            dQ = dQ_one * inp.blades

            reynolds = inp.rho * v_rel * chord / inp.mu

            section_data.append(
                SectionResult(
                    r_m=r,
                    chord_m=chord,
                    beta_deg=math.degrees(beta),
                    phi_deg=math.degrees(phi),
                    alpha_deg=math.degrees(alpha),
                    reynolds=reynolds,
                    cl=cl,
                    cd=cd,
                    dT_N=dT,
                    dQ_Nm=dQ,
                )
            )

            thrust += dT
            torque += dQ

        new_vi = estimate_induced_velocity(max(thrust, 1e-6), inp.rho, A, inp.flight_speed_mps)
        vi = 0.65 * vi + 0.35 * new_vi

    power = torque * omega

    ct = thrust / (inp.rho * n * n * inp.diameter_m ** 4)
    cp = power / (inp.rho * n ** 3 * inp.diameter_m ** 5)
    cq = torque / (inp.rho * n ** 2 * inp.diameter_m ** 5)

    ideal_vi = estimate_induced_velocity(max(thrust, 1e-6), inp.rho, A, inp.flight_speed_mps)
    ideal_power = thrust * (inp.flight_speed_mps + ideal_vi)
    figure_of_merit = ideal_power / power if power > 1e-8 else 0.0

    return PropellerResults(
        inputs=asdict(inp),
        blade_radius_m=R,
        area_disk_m2=A,
        n_rev_per_sec=n,
        sections=section_data,
        thrust_N=thrust,
        torque_Nm=torque,
        power_W=power,
        ct=ct,
        cp=cp,
        cq=cq,
        figure_of_merit=figure_of_merit,
        ideal_induced_velocity_mps=ideal_vi,
        ideal_power_W=ideal_power,
    )


def print_human_summary(results: PropellerResults) -> None:
    print("\n=== Propeller Design + Performance Summary ===")
    print(f"Diameter: {results.inputs['diameter_m']:.4f} m")
    print(f"Blades:   {results.inputs['blades']}")
    print(f"RPM:      {results.inputs['rpm']:.1f}")
    print(f"R:        {results.blade_radius_m:.4f} m")
    print(f"Disk A:   {results.area_disk_m2:.5f} m^2")
    print("---")
    print(f"Total Thrust (T): {results.thrust_N:.3f} N")
    print(f"Total Torque (Q): {results.torque_Nm:.4f} N·m")
    print(f"Shaft Power (P):  {results.power_W:.2f} W")
    print("---")
    print(f"CT: {results.ct:.6f}")
    print(f"CP: {results.cp:.6f}")
    print(f"CQ: {results.cq:.6f}")
    print(f"Figure of Merit: {results.figure_of_merit:.4f}")
    print(f"Induced velocity (ideal): {results.ideal_induced_velocity_mps:.3f} m/s")

    print("\nSection table (first 10 sections):")
    header = (
        "r(m)    chord(m)  beta(deg) phi(deg) alpha(deg) "
        "Re        CL      CD      dT(N)    dQ(Nm)"
    )
    print(header)
    for sec in results.sections[:10]:
        print(
            f"{sec.r_m:0.4f}  {sec.chord_m:0.4f}   {sec.beta_deg:7.3f}  "
            f"{sec.phi_deg:7.3f}   {sec.alpha_deg:8.3f}  {sec.reynolds:8.0f}  "
            f"{sec.cl:6.3f}  {sec.cd:6.4f}  {sec.dT_N:7.4f}  {sec.dQ_Nm:7.5f}"
        )


def to_json_dict(results: PropellerResults) -> Dict[str, Any]:
    data = asdict(results)
    data["sections"] = [asdict(s) for s in results.sections]
    return data


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="UAV propeller preliminary design/performance estimator")
    p.add_argument("--diameter-m", type=float, default=0.254, help="Prop diameter [m]")
    p.add_argument("--hub-radius-m", type=float, default=0.02, help="Hub radius [m]")
    p.add_argument("--blades", type=int, default=2, help="Number of blades")
    p.add_argument("--rpm", type=float, default=6500.0, help="Rotational speed [RPM]")
    p.add_argument("--flight-speed-mps", type=float, default=0.0, help="Axial flight speed [m/s]")
    p.add_argument("--rho", type=float, default=1.225, help="Air density [kg/m^3]")
    p.add_argument("--mu", type=float, default=1.81e-5, help="Dynamic viscosity [Pa*s]")
    p.add_argument("--pitch-at-75-m", type=float, default=0.1143, help="Geometric pitch at 75%R [m]")
    p.add_argument("--root-chord-m", type=float, default=0.028, help="Root chord [m]")
    p.add_argument("--tip-chord-m", type=float, default=0.012, help="Tip chord [m]")
    p.add_argument("--radial-sections", type=int, default=30, help="Number of blade sections")
    p.add_argument("--cl-alpha-per-rad", type=float, default=5.7, help="Lift slope [1/rad]")
    p.add_argument("--cl-max", type=float, default=1.2, help="Max |CL| clipping")
    p.add_argument("--cd0", type=float, default=0.012, help="Profile drag coefficient at zero lift")
    p.add_argument("--k-induced-drag", type=float, default=0.02, help="Quadratic drag factor")
    p.add_argument("--json-out", type=str, default="", help="Optional output JSON file path")
    return p.parse_args()


def validate_inputs(args: argparse.Namespace) -> None:
    if args.diameter_m <= 0:
        raise ValueError("diameter must be > 0")
    if args.hub_radius_m <= 0 or args.hub_radius_m >= args.diameter_m / 2:
        raise ValueError("hub radius must be > 0 and < prop radius")
    if args.blades < 1:
        raise ValueError("blades must be >= 1")
    if args.rpm <= 0:
        raise ValueError("rpm must be > 0")
    if args.radial_sections < 5:
        raise ValueError("radial sections must be >= 5")


def main() -> None:
    args = parse_args()
    validate_inputs(args)

    inputs = PropellerInputs(
        diameter_m=args.diameter_m,
        hub_radius_m=args.hub_radius_m,
        blades=args.blades,
        rpm=args.rpm,
        flight_speed_mps=args.flight_speed_mps,
        rho=args.rho,
        mu=args.mu,
        pitch_at_75_m=args.pitch_at_75_m,
        tip_chord_m=args.tip_chord_m,
        root_chord_m=args.root_chord_m,
        radial_sections=args.radial_sections,
        cl_alpha_per_rad=args.cl_alpha_per_rad,
        cl_max=args.cl_max,
        cd0=args.cd0,
        k_induced_drag=args.k_induced_drag,
    )

    results = solve_propeller(inputs)
    print_human_summary(results)

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump(to_json_dict(results), f, indent=2)
        print(f"\nSaved JSON report to: {args.json_out}")


if __name__ == "__main__":
    main()
