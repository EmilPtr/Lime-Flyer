#!/usr/bin/env python3
"""Citrus Tiltrotor: airframe geometry, mass/CG, stability, hover balance,
clearance and structure checks.

Standard library only. Run:   python3 -I tools/geometry.py
Write to-scale SVG views:     python3 -I tools/geometry.py --svg

Coordinate system (all lengths in mm, masses in g):
  origin = wing leading edge on the aircraft centerline
  x = aft (positive behind the leading edge)
  y = right (positive toward the right wing tip)
  z = up    (z = 0 is the mid-plane of the 5 mm foam wing)

Tags used in the comments:  [A] assumed, [C] calculated, [M] measured.
Nothing in this file is measured yet. Replace [A] values with your own
measurements (scale, thrust stand, balance test) and re-run.

The equations are the same ones used in docs/design-guide.html:
  CG          x_cg = sum(m_i x_i) / sum(m_i)
  tail volume V_H = S_h l_t / (S c),  V_V = S_v l_v / (S b)
              (conventional tail: flat stabilizer + elevator, fixed fin)
  NP (rough)  a = 2 pi / (1 + 2/(e AR)),  de/da = 2a/(pi AR),
              h_np = 0.25 + (a_t/a)(1 - de/da) V_H 0.9
  hover       T_rear = W d_f/(d_f+d_r),  T_front_total = W d_r/(d_f+d_r)
              throttle ~ sqrt(T_needed / T_max)
  clearances  prop-to-wing = (axis ahead of LE) - r, prop-to-prop = 2y - 2r,
              prop-to-body = y - r - w/2
"""
import math
import sys

# =====================================================================
# PARAMETERS (edit these)
# =====================================================================
G = 9.81            # m/s^2
RHO = 1.225         # kg/m^3, sea level 15 C

# ---- wing (user decision) -------------------------------------------
SPAN = 1300.0       # b, mm
CHORD = 200.0       # c, mm, rectangular so MAC = c
FOAM_T = 5.0        # foam board thickness, mm
ROOT_Y = 25.0       # wing root / tube clamp station = fuselage half-width
X_SPAR = 50.0       # spar line, 25 % chord
AIL_Y0, AIL_Y1 = 370.0, 630.0   # aileron span stations (each side)
AIL_CHORD = 50.0                # 25 % chord, hinge at x = 150
CLMAX = 0.9         # [A] foam board, rounded LE, slight camber (guide default)
E_OSW = 0.7         # [A] Oswald / span efficiency (guide default)

# ---- CG target -------------------------------------------------------
X_CG_TARGET = 60.0  # 30 % chord: top of the 25-30 % window, SM inside 5-15 %

# ---- propulsion ------------------------------------------------------
PROP_D = 127.0      # 5 in
R = PROP_D / 2      # 63.5 mm
T_MAX = 680.0       # [A] static thrust per motor on 3S, g (NOT measured)
KV = 2300.0
CELLS = 3
FM = 0.5            # [A] hover figure of merit (guide default)
USABLE = 0.8        # usable battery fraction

# ---- front nacelles --------------------------------------------------
NAC_Y_REC = 260.0   # recommended inboard station (tilt-axis / motor centerline)
NAC_Y_TIP = 650.0   # user's original wing-tip station
X_AXLE = -95.0      # tilt axle x (ahead of the LE)
Z_AXLE = -10.0      # tilt axle z (pylon hangs under the wing)
H_PROP = 45.0       # [A] axle to prop plane along the thrust line
                    #     (15 mm yoke + ~30 mm motor/prop hub)
E_ROT = 25.0        # [A] axle to CG of the tilting group along the thrust line
TILT_MIN, TILT_MAX = -15.0, 95.0  # deg; 0 = thrust straight up, 90 = forward
TILT_YAW = 10.0     # deg differential tilt used for full yaw command

# ---- rear motor, boom, tail -----------------------------------------
X_REAR = 370.0      # rear motor x (chosen = x_cg + 2 d_f)
Z_BOOM = -20.0      # boom centerline z
BOOM_D, BOOM_DI = 10.0, 8.0     # boom tube OD / ID
BOOM_X0, BOOM_X1 = 100.0, 670.0 # boom from inside the fuselage to the tail
REAR_MOUNT_H = 10.0             # boom top to motor mount face
MOTOR_PROP_H = 30.0             # [A] mount face to prop plane (RS2205 + prop nut)
X_TAIL_LE = 560.0   # horizontal stabilizer leading edge
C_TAIL = 110.0      # stabilizer chord (rectangular, elevator included)
B_HT = 400.0        # stabilizer span
ELEV_C = 38.0       # elevator chord (~35 %), one piece, full span
Z_HT = -10.0        # stabilizer lower surface (sits on the tail mount on the boom)
FIN_TE = 680.0      # fin trailing edge, vertical (future rudder hinge line parallel)
FIN_CR = 160.0      # fin root chord (on top of the stabilizer)
FIN_CT = 100.0      # fin tip chord
FIN_H = 220.0       # fin height above the stabilizer top
RUDDER_C = 40.0     # OPTIONAL future rudder chord (not built now)
RUDDER_GAP = 20.0   # rudder bottom edge kept this far above the stabilizer top
FIN_NOTCH = 15.0    # fin root cut away x = 628..680 so the one-piece elevator swings under it
ELEV_MAX = 20.0     # deg max elevator deflection

# ---- fuselage --------------------------------------------------------
FUS_X0, FUS_X1 = -230.0, 190.0  # nose, tail end of the pod
FUS_W, FUS_H = 50.0, 62.0       # outside width and height
FUS_ZTOP = -2.5                 # top of pod = wing lower surface
GEAR_Z = -100.0                 # bottom of the landing legs (ground plane)
GEAR_X_FRONT = -40.0            # front legs under the nacelle pylons
GEAR_X_REAR = 370.0             # rear leg under the rear motor

# ---- materials [A] ---------------------------------------------------
CF_DENS = 1.55e-3   # g/mm^3, roll-wrapped carbon tube
CF_ROD_DENS = 1.60e-3
E_TUBE = 100e3      # N/mm^2 (MPa); roll-wrapped tube, axial
G_TUBE = 12e3       # MPa; roll-wrapped tube (pultruded tubes: ~4e3)
E_ROD = 120e3       # MPa; pultruded rod
G_ROD = 4e3         # MPa; pultruded rod (fibres all axial, weak in shear)
SIG_ALLOW = 500.0   # MPa design allowable in bending (conservative)
E_PAPER = 3e3       # MPa, foam-board paper face
T_PAPER = 0.15      # mm, one paper face
FOAM_AREAL = 0.40e-3  # g/mm^2 bare 5 mm foam board (0.40 kg/m^2). Weigh a sheet!
SHEET = (762.0, 508.0)  # Dollar Tree / Adams 20 x 30 in

# ---- load factors -----------------------------------------------------
N_FLIGHT = 3.0      # gust / pull-up, user's 2-3 g
N_LAND = 3.0        # hard landing on the gear, nacelle inertia
SF = 1.5            # safety factor on stress
F_MIN = 15.0        # Hz, minimum first structural mode (rule of thumb [A])
TWIST_MAX = 2.0     # deg, max nacelle twist at full thrust on one motor
RATE_MAX = 3.0      # rad/s, max body rate for gyroscopic load (about 170 deg/s)

# ---- batteries --------------------------------------------------------
BATT = {"3S 1800": (155.0, 1.8, 100.0), "3S 2200": (190.0, 2.2, 105.0), "3S 1300": (110.0, 1.3, 75.0)}  # g, Ah, length [A] (1800 is an estimate: weigh the real pack)


# =====================================================================
# helpers
# =====================================================================
def tube(do, di, dens=CF_DENS, E=E_TUBE, Gs=G_TUBE):
    I = math.pi * (do**4 - di**4) / 64.0
    return {"name": f"{do:g}x{di:g} tube" if di else f"{do:g} mm rod",
            "I": I, "J": 2 * I, "Z": I / (do / 2), "E": E, "G": Gs,
            "gpm": math.pi / 4 * (do**2 - di**2) * dens}   # g per mm


ROD4 = tube(4, 0, CF_ROD_DENS, E_ROD, G_ROD)
T8 = tube(8, 6)
T10 = tube(10, 8)
T12 = tube(12, 10)


def disc_pts(theta_deg, n=41):
    """Points (x,z) of the front prop disc section at tilt theta (side view)."""
    t = math.radians(theta_deg)
    cx = X_AXLE - H_PROP * math.sin(t)
    cz = Z_AXLE + H_PROP * math.cos(t)
    return [(cx + s * math.cos(t), cz + s * math.sin(t))
            for s in [-R + 2 * R * i / (n - 1) for i in range(n)]]


def hdr(s):
    print("\n" + s + "\n" + "-" * len(s))


# =====================================================================
# 1. wing basics
# =====================================================================
S_mm2 = SPAN * CHORD
S_dm2 = S_mm2 / 1e4
AR = SPAN / CHORD


# =====================================================================
# 2. components (mass g, x, y, z)  -- recommended layout
# =====================================================================
def components(nac_y, battery, x_batt, spar="rec"):
    m_b = BATT[battery][0]
    s_h = B_HT * C_TAIL                        # mm^2
    s_v = (FIN_CR + FIN_CT) / 2 * FIN_H        # mm^2
    x_fin = FIN_TE - (FIN_CR**2 + FIN_CR * FIN_CT + FIN_CT**2) / (3 * (FIN_CR + FIN_CT))
    tube_len = 2 * (nac_y + 40)          # continuous centre tube (rec layout)
    rod_len = 2 * (SPAN / 2 - 10 - rod_y0(nac_y))  # 4 mm rods, 100 mm overlap with tube
    c = [
        # name, mass, x, y, z, tag
        ("Battery " + battery, m_b, x_batt, 0, -35, "A (BOM)"),
        ("Flight controller", 10, 58, 0, -40, "A (BOM)"),
        ("Receiver iA6B", 5, -20, 0, -30, "A (BOM)"),
        ("3x ESC 25 A (nose bay)", 30, -110, 0, -45, "A (BOM)"),
        ("Wiring, BEC, connectors", 40, 0, 0, -30, "A (BOM)"),
        ("Front-left tilting group (motor, prop, yoke)", 42, X_AXLE, -nac_y, Z_AXLE + E_ROT, "A"),
        ("Front-right tilting group", 42, X_AXLE, nac_y, Z_AXLE + E_ROT, "A"),
        ("Front-left tilt servo", 13, -85, -nac_y - 25, Z_AXLE, "A (BOM)"),
        ("Front-right tilt servo", 13, -85, nac_y + 25, Z_AXLE, "A (BOM)"),
        ("Front-left pylon (PETG)", 10, -20, -nac_y, -12, "A"),
        ("Front-right pylon (PETG)", 10, -20, nac_y, -12, "A"),
        ("Rear motor, prop, mount", 42, X_REAR, 0, Z_BOOM + 20, "A"),
        ("Aileron servo L", 9, 110, -500, 0, "A (BOM)"),
        ("Aileron servo R", 9, 110, 500, 0, "A (BOM)"),
        ("Elevator servo (in fuselage)", 9, 150, 0, -30, "A (BOM)"),
        ("Elevator pushrod + guides", 4, 360, 0, Z_BOOM, "A"),
        ("Foam wing, 2 halves (incl. glue, tape)", 130, 100, 0, 0, "A (BOM)"),
    ]
    if spar == "rec":
        c += [("Centre tube 10x8, %d mm" % tube_len, T10["gpm"] * tube_len, X_SPAR, 0, -7.5, "C"),
              ("2x 4 mm rods, %d mm total" % rod_len, ROD4["gpm"] * rod_len, X_SPAR, 0, 0, "C")]
    else:  # tip layout: 12x10 tube each half root->tip + 10x8 joiner
        L = 2 * (SPAN / 2 - ROOT_Y)
        c += [("2x 12x10 spar tubes, %d mm" % L, T12["gpm"] * L, X_SPAR, 0, -8.5, "C"),
              ("10x8 joiner, 200 mm", T10["gpm"] * 200, X_SPAR, 0, -8.5, "C")]
    blen = BOOM_X1 - BOOM_X0
    c += [
        ("Boom 10x8, %d mm" % blen, T10["gpm"] * blen, (BOOM_X0 + BOOM_X1) / 2, 0, Z_BOOM, "C"),
        ("Fuselage pod (PLA)", 85, (FUS_X0 + FUS_X1) / 2 + 10, 0, -33, "A"),
        ("Wing clips + root ribs (PLA)", 15, X_SPAR, 0, -5, "A"),
        ("Boom clamps (PETG)", 10, 145, 0, Z_BOOM, "A"),
        ("Tail mount: stab saddle + fin socket (PLA)", 15, 600, 0, Z_BOOM + 10, "A"),
        ("Landing legs, front pair (PETG)", 14, GEAR_X_FRONT, 0, -60, "A"),
        ("Landing leg, rear (PETG)", 6, GEAR_X_REAR, 0, -60, "A"),
        ("Stabilizer + elevator (foam)", s_h * FOAM_AREAL, X_TAIL_LE + C_TAIL / 2, 0, Z_HT + 2.5, "C"),
        ("Fin (foam)", s_v * FOAM_AREAL, x_fin, 0, Z_HT + 5 + FIN_H * 0.45, "C"),
        ("Horns, hinges, linkages", 6, 300, 0, 0, "A"),
        ("Glue, tape, misc", 14, 100, 0, 0, "A"),
    ]
    return c


def rod_y0(nac_y):
    """Inner end of the 4 mm rod: 100 mm inboard of the centre-tube end."""
    return nac_y + 40 - 100


def cg(comp):
    M = sum(c[1] for c in comp)
    x = sum(c[1] * c[2] for c in comp) / M
    y = sum(c[1] * c[3] for c in comp) / M
    z = sum(c[1] * c[4] for c in comp) / M
    return M, x, y, z


def solve_batt(nac_y, battery, spar="rec"):
    """Battery x that puts the CG on X_CG_TARGET, rounded to 5 mm."""
    comp = components(nac_y, battery, 0.0, spar)
    M, x0, _, _ = cg(comp)
    m_b = BATT[battery][0]
    xb = (X_CG_TARGET * M - x0 * M) / m_b
    return 5 * round(xb / 5)


# =====================================================================
# 3. tail and neutral point (guide method)
# =====================================================================
def fin_geom():
    """Trapezoid fin with vertical TE: area, MAC, MAC height, quarter-MAC x."""
    cr, ct, h = FIN_CR, FIN_CT, FIN_H
    S = (cr + ct) / 2 * h
    lam = ct / cr
    mac = 2 / 3 * cr * (1 + lam + lam**2) / (1 + lam)
    zbar = h / 3 * (1 + 2 * lam) / (1 + lam)
    x_le_root = FIN_TE - cr
    x_le_mac = x_le_root + (cr - ct) * zbar / h
    return dict(S=S / 1e4, mac=mac, zbar=zbar, x_le_root=x_le_root,
                x_le_tip=FIN_TE - ct, xq=x_le_mac + 0.25 * mac)


def tail(x_cg):
    Sh = B_HT * C_TAIL / 1e4               # dm^2
    f = fin_geom()
    Sv = f["S"]
    lt = X_TAIL_LE + 0.25 * C_TAIL - x_cg  # CG to stabilizer quarter-chord
    lv = f["xq"] - x_cg                    # CG to fin quarter-MAC
    VH = Sh * lt / (S_dm2 * CHORD)
    VV = Sv * lv / (S_dm2 * SPAN)
    a = 2 * math.pi / (1 + 2 / (E_OSW * AR))
    de = 2 * a / (math.pi * AR)
    ar_t = B_HT / C_TAIL
    at = 2 * math.pi / (1 + 2 / (E_OSW * ar_t))
    hnp = 0.25 + (at / a) * (1 - de) * VH * 0.9
    return dict(Sh=Sh, Sv=Sv, lt=lt, lv=lv, VH=VH, VV=VV, a=a, de=de,
                ar_t=ar_t, at=at, hnp=hnp, fin=f,
                elev_area=B_HT * ELEV_C / 1e4)


# =====================================================================
# 4. hover
# =====================================================================
def hover(M, x_cg):
    d_f = x_cg - X_AXLE
    d_r = X_REAR - x_cg
    T_r = M * d_f / (d_f + d_r)
    T_ft = M * d_r / (d_f + d_r)
    T_f = T_ft / 2
    thr_f = math.sqrt(T_f / T_MAX)
    thr_r = math.sqrt(T_r / T_MAX)
    return dict(d_f=d_f, d_r=d_r, T_r=T_r, T_f=T_f, thr_f=thr_f, thr_r=thr_r,
                thr_max=max(thr_f, thr_r), TW=3 * T_MAX / M,
                TW_eff=T_MAX / max(T_f, T_r))   # margin of the busiest motor


def hover_power(M, battery):
    A1 = math.pi * (R / 1000) ** 2
    W = M / 1000 * G
    P_id = W ** 1.5 / math.sqrt(2 * RHO * 3 * A1)
    P = P_id / FM
    I = P / (CELLS * 3.7)
    t = 60 * BATT[battery][1] * USABLE / I
    return P, I, t


def inertia(comp, nac_y, x_cg, z_cg):
    """Roll (Ixx), pitch (Iyy), yaw (Izz) about the CG, kg m^2.
    Point masses, except spanwise members (wing, spar, rods) as uniform bars
    and the boom as a uniform bar along x."""
    Ixx = Iyy = Izz = 0.0
    for name, m, x, y, z, _ in comp:
        m_kg = m / 1000
        dx, dy, dz = (x - x_cg) / 1000, y / 1000, (z - z_cg) / 1000
        if name.startswith("Foam wing") or "rods" in name or "spar tubes" in name:
            ly = (SPAN if "wing" in name or "spar" in name else 2 * (SPAN / 2 - 10 - rod_y0(nac_y))) / 1000
            Iy_bar = m_kg * ly ** 2 / 12
            Ixx += Iy_bar + m_kg * dz ** 2
            Izz += Iy_bar + m_kg * dx ** 2
            Iyy += m_kg * (dx ** 2 + dz ** 2)
        elif name.startswith("Centre tube"):
            ly = 2 * (nac_y + 40) / 1000
            Ixx += m_kg * ly ** 2 / 12 + m_kg * dz ** 2
            Izz += m_kg * ly ** 2 / 12 + m_kg * dx ** 2
            Iyy += m_kg * (dx ** 2 + dz ** 2)
        elif name.startswith("Boom"):
            lx = (BOOM_X1 - BOOM_X0) / 1000
            Iyy += m_kg * lx ** 2 / 12 + m_kg * (dx ** 2 + dz ** 2)
            Izz += m_kg * lx ** 2 / 12 + m_kg * dx ** 2
            Ixx += m_kg * dz ** 2
        else:
            Ixx += m_kg * (dy ** 2 + dz ** 2)
            Iyy += m_kg * (dx ** 2 + dz ** 2)
            Izz += m_kg * (dx ** 2 + dy ** 2)
    return Ixx, Iyy, Izz


def control(M, h, nac_y, I):
    Ixx, Iyy, Izz = I
    T_f, T_r = h["T_f"], h["T_r"]
    # roll: one front motor up, other down, total lift unchanged
    dT = min(T_MAX - T_f, T_f)
    M_roll = 2 * dT / 1000 * G * nac_y / 1000
    # pitch: nose-down limit, rear up to T_MAX while front pair keeps lift
    dTr = min(T_MAX - T_r, T_r)
    M_pitch = dTr / 1000 * G * (h["d_r"] + h["d_f"]) / 1000  # moving lift fore/aft
    # yaw: differential tilt TILT_YAW
    M_yaw = 2 * T_f / 1000 * G * math.sin(math.radians(TILT_YAW)) * nac_y / 1000
    # rear prop reaction torque from momentum theory at hover thrust
    A1 = math.pi * (R / 1000) ** 2
    Tn = T_r / 1000 * G
    P_shaft = Tn ** 1.5 / math.sqrt(2 * RHO * A1) / FM
    rpm_max = KV * CELLS * 3.7 * 0.85
    rpm_h = rpm_max * math.sqrt(T_r / T_MAX)
    Q = P_shaft / (rpm_h * 2 * math.pi / 60)
    d_trim = math.degrees(math.asin(Q / (2 * T_f / 1000 * G * nac_y / 1000)))
    return dict(dT=dT, M_roll=M_roll, a_roll=M_roll / Ixx, M_pitch=M_pitch,
                a_pitch=M_pitch / Iyy, M_yaw=M_yaw, a_yaw=M_yaw / Izz,
                Q=Q, rpm_h=rpm_h, rpm_max=rpm_max, d_trim=d_trim)


# =====================================================================
# 5. clearances
# =====================================================================
def clearances(nac_y, x_cg):
    out = {}
    # (a) plan-view prop-to-LE gap in hover (guide definition)
    out["le_plan_hover"] = (0 - X_AXLE) - R
    # (b) worst gap to the wing LE over the whole tilt sweep, side view.
    # LE modelled as the rounded nose: points x=0, z in [-2.5, 2.5].
    le_pts = [(0.0, -FOAM_T / 2), (0.0, 0.0), (0.0, FOAM_T / 2)]
    worst, worst_t, aftmost, aft_t = 1e9, None, -1e9, None
    th = TILT_MIN
    while th <= TILT_MAX + 1e-9:
        pts = disc_pts(th)
        for (px, pz) in pts:
            for (lx, lz) in le_pts:
                d = math.hypot(px - lx, pz - lz)
                if d < worst:
                    worst, worst_t = d, th
            if px > aftmost:
                aftmost, aft_t = px, th
        th += 1.0
    out["le_sweep_min"], out["le_sweep_theta"] = worst, worst_t
    out["le_plan_worst"], out["le_plan_theta"] = -aftmost, aft_t
    out["sweep_radius"] = math.hypot(H_PROP, R)
    # (c) prop to prop, prop to fuselage (guide definitions)
    out["prop_prop"] = 2 * nac_y - 2 * R
    out["prop_body"] = nac_y - R - FUS_W / 2
    # (d) lowest point of the front disc over the sweep vs ground (gear)
    low = min(pz for th in range(int(TILT_MIN), int(TILT_MAX) + 1) for (_, pz) in disc_pts(th))
    out["front_low_z"] = low
    out["front_ground"] = low - GEAR_Z
    # (e) front disc vs front landing leg (leg is a vertical strut at x=-40 under pylon)
    leg = [(GEAR_X_FRONT, z) for z in range(int(GEAR_Z), int(Z_AXLE) - 5, 5)]
    lm = 1e9
    for th in range(int(TILT_MIN), int(TILT_MAX) + 1):
        for (px, pz) in disc_pts(th):
            for (lx, lz) in leg:
                lm = min(lm, math.hypot(px - lx, pz - lz))
    out["front_leg"] = lm
    # (f) rear prop
    z_rear_prop = Z_BOOM + BOOM_D / 2 + REAR_MOUNT_H + MOTOR_PROP_H
    out["rear_prop_z"] = z_rear_prop
    out["rear_boom"] = z_rear_prop - (Z_BOOM + BOOM_D / 2)
    out["rear_tail_plan"] = X_TAIL_LE - (X_REAR + R)
    f = fin_geom()
    out["rear_fin_plan"] = f["x_le_root"] - (X_REAR + R)
    # fin LE at the height of the rear prop plane
    zf0 = Z_HT + FOAM_T
    frac = min(max((z_rear_prop - zf0) / FIN_H, 0), 1)
    out["rear_fin_at_disc_z"] = f["x_le_root"] + (FIN_CR - FIN_CT) * frac - (X_REAR + R)
    out["rear_stab_below"] = z_rear_prop - (Z_HT + FOAM_T)
    out["rear_te_plan"] = (X_REAR - R) - CHORD
    out["rear_ground"] = z_rear_prop - GEAR_Z
    # (g) cruise disc position
    out["cruise_disc_x"] = X_AXLE - H_PROP
    out["cruise_le_gap"] = 0 - (X_AXLE - H_PROP)
    # (h) slipstream band (fully contracted radius r/sqrt2) and ailerons
    rw = R / math.sqrt(2)
    out["wash_band"] = (nac_y - rw, nac_y + rw)
    out["wash_band_full"] = (nac_y - R, nac_y + R)
    out["ail_overlap"] = max(0.0, min(nac_y + R, AIL_Y1) - max(nac_y - R, AIL_Y0))
    return out


# =====================================================================
# 6. structure
# =====================================================================
def wing_struct(nac_y, sec, M, h, m_nac=65.0, m_wing=130.0, label=""):
    """Half-wing / tube as a cantilever from the fuselage side (ROOT_Y) to
    the nacelle station. Loads in N and N mm."""
    L = nac_y - ROOT_Y
    Lw = SPAN / 2 - ROOT_Y
    W = M / 1000 * G
    wn = m_nac / 1000 * G
    ww = m_wing / 2 / 1000 * G
    # bending moments at root (positive = tips up)
    M_hov = (h["T_f"] / 1000 * G - wn) * L - ww * Lw / 2
    M_hmax = (T_MAX / 1000 * G - wn) * L - ww * Lw / 2
    M_cru = N_FLIGHT * (W / 2 * Lw / 2 - wn * L - ww * Lw / 2)
    M_land = -N_LAND * (wn * L + ww * Lw / 2)
    M_design = max(abs(M_hov), abs(M_hmax), abs(M_cru), abs(M_land))
    EI = sec["E"] * sec["I"]
    GJ = sec["G"] * sec["J"]
    sigma = SF * M_design / sec["Z"]
    # tip deflection under point load at nacelle (hover)
    P_hov = h["T_f"] / 1000 * G - wn
    P_max = T_MAX / 1000 * G - wn
    d_hov = P_hov * L ** 3 / (3 * EI)
    d_max = P_max * L ** 3 / (3 * EI)
    # torsion: thrust line at axle, spar at X_SPAR
    arm = X_SPAR - X_AXLE
    Tq = T_MAX / 1000 * G * arm
    twist = math.degrees(Tq * L / GJ)
    # modes (single degree of freedom)
    outboard = m_wing / 2 * (SPAN / 2 - nac_y) / Lw + (9 if nac_y < AIL_Y1 else 0)
    m_eff = m_nac + 0.24 * m_wing / 2 * L / Lw + outboard
    k = 3 * EI / L ** 3                         # N/mm
    f_b = math.sqrt(k * 1000 / (m_eff / 1000)) / (2 * math.pi)
    # nacelle pitch inertia about spar axis (kg m^2)
    I_n = (0.042 * ((X_SPAR - X_AXLE) ** 2 + E_ROT ** 2) + 0.013 * (X_SPAR + 85) ** 2
           + 0.010 * (X_SPAR + 20) ** 2) / 1e6
    K_t = GJ / L / 1000                         # N m / rad
    f_t = math.sqrt(K_t / I_n) / (2 * math.pi)
    return dict(label=label, L=L, M_hov=M_hov, M_hmax=M_hmax, M_cru=M_cru, M_land=M_land,
                M_design=M_design, sigma=sigma, d_hov=d_hov, d_max=d_max, Tq=Tq,
                twist=twist, f_b=f_b, f_t=f_t, EI=EI, GJ=GJ, m_eff=m_eff, I_n=I_n,
                ok=(sigma <= SIG_ALLOW and f_b >= F_MIN and f_t >= F_MIN and twist <= TWIST_MAX))


def foam_EI():
    d = FOAM_T - T_PAPER
    return E_PAPER * 2 * CHORD * T_PAPER * (d / 2) ** 2


def outer_panel(M, nac_y):
    """Rod-only outer panel from the tube end (nac_y + 40) to the tip."""
    y0 = nac_y + 40
    Lo = SPAN / 2 - y0
    Lw = SPAN / 2 - ROOT_Y
    W = M / 1000 * G
    w = N_FLIGHT * W / 2 / Lw                 # N/mm lift
    wself = N_FLIGHT * (130 / 2 / Lw) / 1000 * G
    Mo = (w - wself) * Lo ** 2 / 2 - N_FLIGHT * 0.009 * G * (500 - y0)
    EI = ROD4["E"] * ROD4["I"] + foam_EI()
    sigma = SF * Mo / ROD4["Z"]
    d1 = (w / N_FLIGHT) * Lo ** 4 / (8 * EI)
    m_l = (130 / 2 / Lw + ROD4["gpm"]) / 1000 / 1000  # kg/mm -> kg/mm
    # first cantilever mode, uniform beam: f = 1.875^2/(2 pi L^2) sqrt(EI/m')
    EI_SI = EI * 1e-6          # N m^2
    m_SI = (130 / 2 / Lw + ROD4["gpm"]) / 1000 * 1000   # kg/m
    f = 1.875 ** 2 / (2 * math.pi * (Lo / 1000) ** 2) * math.sqrt(EI_SI / m_SI)
    return dict(y0=y0, Lo=Lo, Mo=Mo, sigma=sigma, d1=d1, f=f, EI=EI)


def boom_struct(M):
    L_r = X_REAR - FUS_X1
    L_t = X_TAIL_LE + 0.25 * C_TAIL - FUS_X1
    EI = T10["E"] * T10["I"]
    M_r = T_MAX / 1000 * G * L_r
    M_t = 3.0 * L_t                       # [A] 3 N tail load at high speed
    sigma = SF * (M_r + M_t) / T10["Z"]
    m_tail = (15 + (B_HT * C_TAIL + (FIN_CR + FIN_CT) / 2 * FIN_H) * FOAM_AREAL + 3) / 1000
    k = 3 * EI / L_t ** 3 * 1000
    f = math.sqrt(k / m_tail) / (2 * math.pi)
    return dict(L_r=L_r, L_t=L_t, M_r=M_r, M_t=M_t, sigma=sigma, f=f)


def tilt_servo(h, ctl):
    m_rot = 0.042
    tq_g = m_rot * G * E_ROT / 1000
    I_p = 0.004 * (PROP_D / 1000) ** 2 / 12 + 0.015 * 0.0135 ** 2  # [A] prop + bell
    w = ctl["rpm_max"] * 2 * math.pi / 60 * math.sqrt(h["T_f"] / T_MAX)   # front prop, rad/s
    tq_gyro = I_p * w * RATE_MAX
    tq_aero = 0.10 * T_MAX / 1000 * G * H_PROP / 1000      # [A] 10 % in-plane force
    tot = tq_g + tq_gyro + tq_aero
    return dict(tq_g=tq_g, I_p=I_p, w=w, tq_gyro=tq_gyro, tq_aero=tq_aero, tot=tot,
                need_kgcm=2 * tot / G * 100, travel=TILT_MAX - TILT_MIN)


# =====================================================================
# 7. foam nesting
# =====================================================================
def nesting():
    """Two sheets: sheet 1 = wing + doublers, sheet 2 = tail + spares."""
    Lx, Ly = SHEET
    s1 = [("Wing half L (aileron cut in)", SPAN / 2, CHORD, 0, 0),
          ("Wing half R (aileron cut in)", SPAN / 2, CHORD, 0, CHORD + 3),
          ("Root doubler L", 140, 100, 0, 2 * CHORD + 6),
          ("Root doubler R", 140, 100, 143, 2 * CHORD + 6),
          ("Aileron-servo doubler L", 60, 60, 286, 2 * CHORD + 6),
          ("Aileron-servo doubler R", 60, 60, 349, 2 * CHORD + 6),
          ("Nacelle-station doubler L", 100, 80, 412, 2 * CHORD + 6),
          ("Nacelle-station doubler R", 100, 80, 515, 2 * CHORD + 6)]
    s2 = [("Stabilizer (elevator cut in)", B_HT, C_TAIL, 0, 0),
          ("Fin (trapezoid, bounding box)", FIN_CR, FIN_H, B_HT + 3, 0),
          ("Spare stabilizer", B_HT, C_TAIL, 0, C_TAIL + 3),
          ("Spare fin (bounding box)", FIN_CR, FIN_H, B_HT + FIN_CR + 6, 0),
          ("Spare wing-tip / doubler stock", 762 - 3, 508 - FIN_H - 6, 0, FIN_H + 6)]
    res = []
    for sheet in (s1, s2):
        ok = all(x + w <= Lx and y + h <= Ly for (_, w, h, x, y) in sheet)
        used = sum(w * h for (n, w, h, _, _) in sheet if not n.startswith("Spare wing"))
        res.append((sheet, ok, used / (Lx * Ly)))
    return res


# =====================================================================
# main report
# =====================================================================
def main():
    print("Citrus Tiltrotor geometry check  (tools/geometry.py)")
    print("All values are calculated from the [A]ssumed inputs at the top of the file.")

    hdr("1. Wing")
    W850 = 0.850 * G
    vs = math.sqrt(2 * W850 / (RHO * S_mm2 / 1e6 * CLMAX))
    print(f"S = {S_dm2:.1f} dm^2, AR = {AR:.2f}, chord {CHORD:.0f} mm, span {SPAN:.0f} mm")
    print(f"Spar line x = {X_SPAR:.0f} mm (25 % c). Ailerons y = {AIL_Y0:.0f}..{AIL_Y1:.0f} mm, "
          f"chord {AIL_CHORD:.0f} mm, hinge x = {CHORD - AIL_CHORD:.0f} mm, "
          f"area {2 * (AIL_Y1 - AIL_Y0) * AIL_CHORD / 1e4:.2f} dm^2 "
          f"({2 * (AIL_Y1 - AIL_Y0) * AIL_CHORD / S_mm2 * 100:.0f} % of S)")

    results = {}
    for batt in BATT:
        xb = solve_batt(NAC_Y_REC, batt)
        comp = components(NAC_Y_REC, batt, xb)
        M, xc, yc, zc = cg(comp)
        results[batt] = (xb, comp, M, xc, zc)

    hdr("2. Component placement and CG  (recommended layout, nacelles at y = +/-%d)" % NAC_Y_REC)
    xb, comp, M, xc, zc = results["3S 1800"]
    print(f"{'component':46s} {'m g':>6s} {'x':>6s} {'y':>6s} {'m*x':>9s}  tag")
    for name, m, x, y, z, tag in comp:
        print(f"{name:46s} {m:6.1f} {x:6.0f} {y:6.0f} {m * x:9.0f}  {tag}")
    print(f"{'TOTAL':46s} {M:6.1f} {'':6s} {'':6s} {sum(c[1] * c[2] for c in comp):9.0f}")
    print(f"x_cg = {xc:.1f} mm = {xc / CHORD * 100:.1f} % chord, y_cg = 0 (symmetric), "
          f"z_cg = {zc:.0f} mm")
    bom_carbon = 40.0
    carbon = sum(c[1] for c in comp if "tube" in c[0] or "rods" in c[0] or c[0].startswith("Boom"))
    print(f"Carbon: {carbon:.1f} g vs BOM 40 g  ->  +{carbon - bom_carbon:.1f} g")
    for batt, (xb, comp_b, Mb, xcb, _) in results.items():
        print(f"  {batt}: battery centre x = {xb:.0f} mm, AUW = {Mb:.0f} g, "
              f"x_cg = {xcb:.1f} mm ({xcb / CHORD * 100:.1f} %)")
    # 1300 pack left at the 2200 station
    xb22 = results["3S 1800"][0]
    comp_x = components(NAC_Y_REC, "3S 1300", xb22)
    Mx, xcx, _, _ = cg(comp_x)
    print(f"  3S 1300 left at x = {xb22:.0f}: x_cg = {xcx:.1f} mm ({xcx / CHORD * 100:.1f} %)  <- move it!")
    # tilt CG shift
    dx_tilt = -2 * 42 * E_ROT / M
    dz_tilt = -2 * 42 * E_ROT / M
    print(f"CG shift hover -> cruise (front motors tilt forward): dx = {dx_tilt:.1f} mm, "
          f"dz = {dz_tilt:.1f} mm")

    hdr("3. Tail, neutral point, static margin")
    t = tail(X_CG_TARGET)
    f = t["fin"]
    print(f"Stabilizer {B_HT:.0f} x {C_TAIL:.0f} mm, LE x = {X_TAIL_LE:.0f}, S_h = {t['Sh']:.2f} dm^2 "
          f"({t['Sh'] / S_dm2 * 100:.1f} % of S), AR {t['ar_t']:.2f}; elevator {ELEV_C:.0f} mm chord, "
          f"{t['elev_area']:.2f} dm^2 ({ELEV_C / C_TAIL * 100:.0f} %)")
    print(f"Fin root {FIN_CR:.0f} / tip {FIN_CT:.0f} / height {FIN_H:.0f} mm, LE x {f['x_le_root']:.0f} (root) "
          f"-> {f['x_le_tip']:.0f} (tip), TE x {FIN_TE:.0f} vertical; S_v = {t['Sv']:.2f} dm^2, "
          f"MAC {f['mac']:.0f} mm at {f['zbar']:.0f} mm up, quarter-MAC x = {f['xq']:.0f}")
    print(f"  l_t = {t['lt']:.0f} mm (to stab quarter-chord x = {X_TAIL_LE + 0.25 * C_TAIL:.1f}), "
          f"l_v = {t['lv']:.0f} mm")
    print(f"  V_H = {t['VH']:.3f}  (target >= 0.40: {'PASS' if t['VH'] >= 0.4 else 'FAIL'})")
    print(f"  V_V = {t['VV']:.4f} (target >= 0.040: {'PASS' if t['VV'] >= 0.04 else 'FAIL'})")
    print(f"  a = {t['a']:.3f} /rad, de/da = {t['de']:.3f}, a_t = {t['at']:.3f} /rad")
    print(f"  h_np = {t['hnp']:.3f} -> x_np = {t['hnp'] * CHORD:.0f} mm")
    zr = Z_BOOM + BOOM_D / 2 + REAR_MOUNT_H + MOTOR_PROP_H
    print(f"  Optional rudder: hinge x = {FIN_TE - RUDDER_C:.0f}, bottom edge z = "
          f"{Z_HT + FOAM_T + RUDDER_GAP:.0f}; elevator TE rises {ELEV_C * math.sin(math.radians(ELEV_MAX)):.0f} mm "
          f"at {ELEV_MAX:.0f} deg up -> {'clear' if ELEV_C * math.sin(math.radians(ELEV_MAX)) < RUDDER_GAP else 'CLASH'}")
    print(f"  Fin-root notch {FIN_NOTCH:.0f} mm high over the elevator: "
          f"{'clear' if ELEV_C * math.sin(math.radians(ELEV_MAX)) < FIN_NOTCH else 'CLASH'}")
    for batt, (xb, _, Mb, xcb, _) in results.items():
        sm = t["hnp"] - xcb / CHORD
        print(f"  {batt}: SM = {sm * 100:.1f} % {'PASS' if 0.05 <= sm <= 0.15 else 'CHECK'}")
    lo = (t["hnp"] - 0.15) * CHORD
    hi = (t["hnp"] - 0.05) * CHORD
    print(f"  CG range for SM 5-15 %: x = {lo:.0f} .. {hi:.0f} mm "
          f"({lo / CHORD * 100:.0f} .. {hi / CHORD * 100:.0f} % c); "
          f"design CG {X_CG_TARGET:.0f} mm")

    hdr("4. Hover balance")
    print(f"Front axle x = {X_AXLE:.0f}, rear motor x = {X_REAR:.0f}, T_max/motor = {T_MAX:.0f} g [A]")
    hov = {}
    for batt, (xb, comp_b, Mb, xcb, zcb) in results.items():
        h = hover(Mb, xcb)
        hov[batt] = h
        P, I, tmin = hover_power(Mb, batt)
        print(f"  {batt}: W = {Mb:.0f} g, d_f = {h['d_f']:.0f}, d_r = {h['d_r']:.0f} "
              f"(ratio {h['d_r'] / h['d_f']:.2f})")
        print(f"     T_rear = {h['T_r']:.0f} g, T_front = 2 x {h['T_f']:.0f} g "
              f"(check sum {h['T_r'] + 2 * h['T_f']:.0f} g)")
        print(f"     throttle front {h['thr_f'] * 100:.0f} %, rear {h['thr_r'] * 100:.0f} %; "
              f"T/W = {h['TW']:.2f} ({'PASS' if h['TW'] >= 2.4 else 'FAIL'} vs 2.4); "
              f"busiest-motor margin {h['TW_eff']:.2f}")
        print(f"     hover power ~{P:.0f} W, ~{I:.1f} A, hover time ~{tmin:.1f} min "
              f"(real ~20 % less)")
        print(f"     thrust/motor needed for T/W 2.4: {2.4 * Mb / 3:.0f} g, for 2.5: {2.5 * Mb / 3:.0f} g")
    h = hover(Mx, xcx)
    print(f"  3S 1300 NOT moved (x_cg {xcx:.0f}): T_rear {h['T_r']:.0f} g, front 2 x {h['T_f']:.0f} g, "
          f"rear throttle {h['thr_r'] * 100:.0f} %")
    h_tip = hover(results["3S 1800"][2], results["3S 1800"][3])

    hdr("5. Inertia and hover control authority (3S 1800 case)")
    for lab, ny, spar in ((f"recommended y={NAC_Y_REC:.0f}", NAC_Y_REC, "rec"), ("wing tips y=650", NAC_Y_TIP, "tip")):
        xb = solve_batt(ny, "3S 1800", spar)
        cp = components(ny, "3S 1800", xb, spar)
        Mi, xi, _, zi = cg(cp)
        I = inertia(cp, ny, xi, zi)
        hh = hover(Mi, xi)
        c = control(Mi, hh, ny, I)
        print(f"  {lab}: AUW {Mi:.0f} g, Ixx {I[0]:.4f}, Iyy {I[1]:.4f}, Izz {I[2]:.4f} kg m^2")
        print(f"     roll: dT {c['dT']:.0f} g/side -> {c['M_roll']:.2f} N m, {c['a_roll']:.0f} rad/s^2")
        print(f"     pitch: {c['M_pitch']:.2f} N m, {c['a_pitch']:.0f} rad/s^2")
        print(f"     yaw (+/-{TILT_YAW:.0f} deg tilt): {c['M_yaw']:.3f} N m, {c['a_yaw']:.1f} rad/s^2; "
              f"rear-prop torque {c['Q'] * 1000:.0f} N mm at {c['rpm_h']:.0f} rpm "
              f"-> trim tilt {c['d_trim']:.2f} deg")
        if spar == "rec":
            ctl_rec = c

    hdr("6. Clearances (recommended layout)")
    cl = clearances(NAC_Y_REC, X_CG_TARGET)
    print(f"  Hover prop to wing LE, plan view (guide formula): {cl['le_plan_hover']:.1f} mm "
          f"({'PASS' if cl['le_plan_hover'] >= 20 else 'FAIL'} >= 20)")
    print(f"  Worst plan-view gap over tilt {TILT_MIN:.0f}..{TILT_MAX:.0f} deg: "
          f"{cl['le_plan_worst']:.1f} mm at {cl['le_plan_theta']:.0f} deg "
          f"({'PASS' if cl['le_plan_worst'] >= 20 else 'FAIL'} >= 20)")
    print(f"  Worst true (side-view) distance disc-to-LE: {cl['le_sweep_min']:.1f} mm at "
          f"{cl['le_sweep_theta']:.0f} deg; sweep radius about axle {cl['sweep_radius']:.1f} mm")
    print(f"  Cruise disc plane x = {cl['cruise_disc_x']:.0f} mm ({cl['cruise_le_gap']:.0f} mm ahead of LE)")
    print(f"  Prop to prop: {cl['prop_prop']:.0f} mm; prop to fuselage side: {cl['prop_body']:.1f} mm")
    print(f"  Front disc lowest point over sweep z = {cl['front_low_z']:.1f}; "
          f"ground clearance {cl['front_ground']:.1f} mm; to front leg {cl['front_leg']:.1f} mm")
    print(f"  Rear prop plane z = {cl['rear_prop_z']:.0f}; to boom top {cl['rear_boom']:.0f} mm; "
          f"to wing TE (plan) {cl['rear_te_plan']:.1f} mm; above ground {cl['rear_ground']:.0f} mm")
    print(f"  Rear disc to stabilizer LE (plan) {cl['rear_tail_plan']:.1f} mm, stab top {cl['rear_stab_below']:.0f} mm "
          f"below the disc; to fin LE root (plan) {cl['rear_fin_plan']:.1f} mm, at disc height "
          f"{cl['rear_fin_at_disc_z']:.1f} mm")
    print(f"  Cruise slipstream band y = {cl['wash_band'][0]:.0f}..{cl['wash_band'][1]:.0f} "
          f"(disc {cl['wash_band_full'][0]:.0f}..{cl['wash_band_full'][1]:.0f}); "
          f"aileron overlap {cl['ail_overlap']:.0f} mm")
    clt = clearances(NAC_Y_TIP, X_CG_TARGET)
    print(f"  Tip layout: prop to prop {clt['prop_prop']:.0f} mm, wash y = {clt['wash_band_full'][0]:.0f}"
          f"..{clt['wash_band_full'][1]:.0f}, {SPAN / 2 - clt['wash_band_full'][0]:.0f} mm of the "
          f"wing tip inside the disc; ailerons to {AIL_Y1:.0f} would overlap "
          f"{max(0, AIL_Y1 - clt['wash_band_full'][0]):.0f} mm")

    hdr("7. Structure")
    M22 = results["3S 1800"][2]
    h22 = hov["3S 1800"]
    print(f"  Foam-board-only EI (paper faces, [A] E={E_PAPER:.0f} MPa, t={T_PAPER} mm): "
          f"{foam_EI():.2e} N mm^2")
    rows = [("tips y=650", NAC_Y_TIP, s) for s in (ROD4, T10, T12)]
    rows += [(f"inboard y={NAC_Y_REC:.0f}", NAC_Y_REC, s) for s in (ROD4, T8, T10)]
    print(f"  {'layout':14s} {'spar':11s} {'Mdes Nm':>7s} {'sig MPa':>7s} {'d_hov':>6s} "
          f"{'d_max':>6s} {'twist':>6s} {'f_bend':>6s} {'f_tors':>6s}  verdict")
    for lab, ny, s in rows:
        sec = dict(s)
        if s is ROD4:
            sec = dict(s, I=s["I"] + foam_EI() / s["E"])  # add foam faces in bending
        r = wing_struct(ny, sec, M22, h22, label=lab)
        print(f"  {lab:14s} {s['name']:11s} {r['M_design'] / 1000:7.2f} {r['sigma']:7.0f} "
              f"{r['d_hov']:6.1f} {r['d_max']:6.1f} {r['twist']:6.1f} {r['f_b']:6.1f} "
              f"{r['f_t']:6.1f}  {'PASS' if r['ok'] else 'FAIL'}")
    r = wing_struct(NAC_Y_REC, T10, M22, h22)
    print(f"  Recommended (y={NAC_Y_REC:.0f}, 10x8): L = {r['L']:.0f} mm, M hover {r['M_hov'] / 1000:.2f}, "
          f"hover-max {r['M_hmax'] / 1000:.2f}, cruise {N_FLIGHT:.0f} g {r['M_cru'] / 1000:.2f}, "
          f"landing {N_LAND:.0f} g {r['M_land'] / 1000:.2f} N m")
    print(f"     torque at full thrust {r['Tq'] / 1000:.2f} N m (arm {X_SPAR - X_AXLE:.0f} mm), "
          f"nacelle pitch inertia {r['I_n'] * 1e6:.0f} kg mm^2, m_eff {r['m_eff']:.0f} g")
    rt = wing_struct(NAC_Y_TIP, T12, M22, h22)
    print(f"  Tips (y=650, 12x10): L = {rt['L']:.0f} mm, M design {rt['M_design'] / 1000:.2f} N m, "
          f"torque {rt['Tq'] / 1000:.2f} N m")
    o = outer_panel(M22, NAC_Y_REC)
    print(f"  Outer panel (rod only, y {o['y0']:.0f}..{SPAN / 2:.0f}, {o['Lo']:.0f} mm): "
          f"M {o['Mo'] / 1000:.2f} N m at {N_FLIGHT:.0f} g, sigma x SF = {o['sigma']:.0f} MPa, "
          f"1 g tip deflection {o['d1']:.1f} mm, f1 = {o['f']:.1f} Hz")
    b = boom_struct(M22)
    print(f"  Boom 10x8: rear thrust arm {b['L_r']:.0f} mm ({b['M_r'] / 1000:.2f} N m), tail arm "
          f"{b['L_t']:.0f} mm, sigma x SF = {b['sigma']:.0f} MPa, tail bending mode {b['f']:.0f} Hz")
    ts = tilt_servo(hov["3S 1800"], ctl_rec)
    print(f"  Tilt servo: gravity {ts['tq_g'] * 1000:.0f}, gyro {ts['tq_gyro'] * 1000:.0f} "
          f"(I_p {ts['I_p'] * 1e6:.1f} kg mm^2, {ts['w']:.0f} rad/s, {RATE_MAX} rad/s body rate), "
          f"aero {ts['tq_aero'] * 1000:.0f} N mm; total {ts['tot'] * 1000:.0f} N mm")
    print(f"     with x2 margin: {ts['need_kgcm']:.2f} kg cm; travel needed {ts['travel']:.0f} deg")

    hdr("8. Mass comparison of layouts (3S 1800 / 2200 / 1300)")
    for lab, ny, spar in ((f"recommended y={NAC_Y_REC:.0f}", NAC_Y_REC, "rec"), ("wing tips y=650", NAC_Y_TIP, "tip")):
        out = []
        for batt in BATT:
            xb = solve_batt(ny, batt, spar)
            Mi, xi, _, _ = cg(components(ny, batt, xb, spar))
            out.append(f"{batt} {Mi:.0f} g T/W {3 * T_MAX / Mi:.2f} batt x {xb:.0f}")
        print(f"  {lab:18s}: " + " | ".join(out))

    hdr("9. Foam board nesting (sheet %.0f x %.0f mm)" % SHEET)
    for i, (sheet, ok, frac) in enumerate(nesting(), 1):
        print(f"  Sheet {i}: fits {'YES' if ok else 'NO'}, {frac * 100:.0f} % used by parts")
        for (n, w, hgt, x, y) in sheet:
            print(f"    {n:32s} {w:5.0f} x {hgt:4.0f} at ({x:4.0f}, {y:4.0f})")

    if "--svg" in sys.argv:
        write_svgs(results)


# =====================================================================
# SVG views (CAD-style, dimensioned)
# =====================================================================
SVG_STYLE = (
    "<style>"
    ".o{fill:none;stroke:#222;stroke-width:1.2}.w{fill:#f4e3b0;stroke:#222;stroke-width:1.2}"
    ".p{fill:#cfe3ff;fill-opacity:.5;stroke:#2a62b8;stroke-width:1}"
    ".c{fill:none;stroke:#888;stroke-width:.8;stroke-dasharray:6 4}"
    ".b{fill:#d9d9d9;stroke:#222;stroke-width:1}.r{fill:#ffd2c2;stroke:#b33;stroke-width:1}"
    ".t{font:12px sans-serif;fill:#222}.cg{fill:#c00}"
    ".g{stroke:#c77;stroke-width:.5;stroke-dasharray:10 3 2 3;fill:none}"
    ".d{stroke:#0b4f8a;stroke-width:.8;fill:none}.da{fill:#0b4f8a}"
    ".dt{font:11.5px sans-serif;fill:#0b4f8a}.dh{font:11.5px sans-serif;fill:#fff;stroke:#fff;stroke-width:3.5px;stroke-linejoin:round}"
    ".fr{fill:none;stroke:#222;stroke-width:1.5}.tb{font:11px sans-serif;fill:#222}"
    ".tbh{font:bold 14px sans-serif;fill:#222}"
    "</style>")


def _n(v):
    return f"{v:.1f}".rstrip("0").rstrip(".") if abs(v - round(v)) > 1e-6 else f"{round(v):d}"


class _Dwg:
    """Tiny SVG builder with CAD dimension helpers. Coordinates are SVG user units (= mm)."""
    AH, AW = 8.0, 2.4  # arrow length, half width

    def __init__(self):
        self.s = []

    def add(self, x):
        self.s.append(x)

    def arrow(self, x, y, dx, dy):
        px, py = -dy, dx
        bx, by = x - dx * self.AH, y - dy * self.AH
        self.add(f'<polygon class="da" points="{x:.1f},{y:.1f} {bx + px * self.AW:.1f},{by + py * self.AW:.1f} '
                 f'{bx - px * self.AW:.1f},{by - py * self.AW:.1f}"/>')

    def text(self, x, y, s, cls="t", anchor="middle", rot=None):
        tr = f' transform="rotate({rot} {x:.1f} {y:.1f})"' if rot is not None else ""
        if cls == "dt":  # white halo drawn first, then the text on top
            self.add(f'<text class="dh" x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}"{tr}>{s}</text>')
        self.add(f'<text class="{cls}" x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}"{tr}>{s}</text>')

    def hdim(self, a, b, y, label=None, e0=None, e1=None, tp="c"):
        """Horizontal dimension between x=a and x=b, dim line at svg y. e0/e1: y where the
        extension line starts (at the feature)."""
        if a > b:
            a, b = b, a
            e0, e1 = e1, e0
        label = _n(b - a) if label is None else label
        for xx, e in ((a, e0), (b, e1)):
            if e is None:
                e = y - 6
            sg = 1 if y > e else -1
            self.add(f'<line class="d" x1="{xx:.1f}" y1="{e + 2 * sg:.1f}" x2="{xx:.1f}" y2="{y + 5 * sg:.1f}"/>')
        narrow = (b - a) < 3 * self.AH + 6
        if narrow:
            self.add(f'<line class="d" x1="{a - 14:.1f}" y1="{y}" x2="{b + 14:.1f}" y2="{y}"/>')
        else:
            self.add(f'<line class="d" x1="{a:.1f}" y1="{y}" x2="{b:.1f}" y2="{y}"/>')
            self.add(f'<polygon class="da" points="{a:.1f},{y} {a + self.AH:.1f},{y - self.AW} {a + self.AH:.1f},{y + self.AW}"/>')
            self.add(f'<polygon class="da" points="{b:.1f},{y} {b - self.AH:.1f},{y - self.AW} {b - self.AH:.1f},{y + self.AW}"/>')
        if narrow:
            # arrows from outside, pointing at the extension lines
            self.add(f'<polygon class="da" points="{a:.1f},{y} {a - self.AH:.1f},{y - self.AW} {a - self.AH:.1f},{y + self.AW}"/>')
            self.add(f'<polygon class="da" points="{b:.1f},{y} {b + self.AH:.1f},{y - self.AW} {b + self.AH:.1f},{y + self.AW}"/>')
        if tp == "l":
            self.text(a - 18, y + 4, label, "dt", "end")
        elif tp == "r":
            self.text(b + 18, y + 4, label, "dt", "start")
        else:
            self.text((a + b) / 2, y - 4, label, "dt")

    def vdim(self, x, a, b, label=None, e0=None, e1=None, side="l"):
        """Vertical dimension between svg y=a and y=b, dim line at svg x. e0/e1: x where the
        extension line starts."""
        if a > b:
            a, b = b, a
            e0, e1 = e1, e0
        label = _n(b - a) if label is None else label
        for yy, e in ((a, e0), (b, e1)):
            if e is None:
                e = x - 6
            sg = 1 if x > e else -1
            self.add(f'<line class="d" x1="{e + 2 * sg:.1f}" y1="{yy:.1f}" x2="{x + 5 * sg:.1f}" y2="{yy:.1f}"/>')
        narrow = (b - a) < 3 * self.AH + 6
        if narrow:
            self.add(f'<line class="d" x1="{x}" y1="{a - 14:.1f}" x2="{x}" y2="{b + 14:.1f}"/>')
            self.add(f'<polygon class="da" points="{x},{a:.1f} {x - self.AW},{a - self.AH:.1f} {x + self.AW},{a - self.AH:.1f}"/>')
            self.add(f'<polygon class="da" points="{x},{b:.1f} {x - self.AW},{b + self.AH:.1f} {x + self.AW},{b + self.AH:.1f}"/>')
        else:
            self.add(f'<line class="d" x1="{x}" y1="{a:.1f}" x2="{x}" y2="{b:.1f}"/>')
            self.add(f'<polygon class="da" points="{x},{a:.1f} {x - self.AW},{a + self.AH:.1f} {x + self.AW},{a + self.AH:.1f}"/>')
            self.add(f'<polygon class="da" points="{x},{b:.1f} {x - self.AW},{b - self.AH:.1f} {x + self.AW},{b - self.AH:.1f}"/>')
        tx = x - 4 if side == "l" else x + 13
        self.text(tx, (a + b) / 2, label, "dt", "middle", rot=-90)

    def hchain(self, pts, y, starts, **kw):
        for i in range(len(pts) - 1):
            self.hdim(pts[i], pts[i + 1], y, e0=starts[i], e1=starts[i + 1], **kw)

    def vchain(self, pts, x, starts, **kw):
        for i in range(len(pts) - 1):
            self.vdim(x, pts[i], pts[i + 1], e0=starts[i], e1=starts[i + 1], **kw)

    def frame(self, vb, title, sub, notes):
        x0, y0, W, H = vb
        self.add(f'<rect class="fr" x="{x0 + 8}" y="{y0 + 8}" width="{W - 16}" height="{H - 16}"/>')
        bw, bh = 470, 78
        bx, by = x0 + W - 8 - bw, y0 + H - 8 - bh
        self.add(f'<rect class="fr" x="{bx}" y="{by}" width="{bw}" height="{bh}" style="fill:#fff"/>')
        self.add(f'<line class="fr" x1="{bx}" y1="{by + 30}" x2="{bx + bw}" y2="{by + 30}" style="stroke-width:1"/>')
        self.add(f'<line class="fr" x1="{bx + 250}" y1="{by + 30}" x2="{bx + 250}" y2="{by + bh}" style="stroke-width:1"/>')
        self.text(bx + 10, by + 21, title, "tbh", "start")
        self.text(bx + 465, by + 21, sub, "tb", "end")
        for i, n in enumerate(notes[0]):
            self.text(bx + 10, by + 47 + 15 * i, n, "tb", "start")
        for i, n in enumerate(notes[1]):
            self.text(bx + 260, by + 47 + 15 * i, n, "tb", "start")


def write_svgs(results):
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "..", "docs")
    xb22 = results["3S 1800"][0]
    xb13 = results["3S 1300"][0]
    ny = NAC_Y_REC
    zt = Z_HT + FOAM_T                       # stabilizer top
    zr = Z_BOOM + BOOM_D / 2 + REAR_MOUNT_H + MOTOR_PROP_H
    fin_top = zt + FIN_H
    X_FIN0 = FIN_TE - FIN_CR
    X_TAIL_TE = X_TAIL_LE + C_TAIL
    X_HINGE = CHORD - AIL_CHORD
    notes0 = ["Units: mm. 1 unit = 1 mm, drawn 1:1 px",
              "Parameters: tools/geometry.py (mostly [A])"]
    notes1 = ["Origin: wing LE on centerline", "x aft, y right, z up; z=0 foam mid-plane"]

    # =============== TOP VIEW: svg X = y, svg Y = x (aft is down, nose up) ===============
    d = _Dwg()
    vb = (-800, -345, 1640, 1150)
    d.add(f'<rect x="{vb[0]}" y="{vb[1]}" width="{vb[2]}" height="{vb[3]}" fill="#fff"/>')
    # datum guides across the drawing
    for gx in (X_AXLE, 0, X_SPAR, X_CG_TARGET, X_HINGE, CHORD, X_REAR, X_TAIL_LE, X_TAIL_TE):
        d.add(f'<line class="g" x1="-725" y1="{gx}" x2="725" y2="{gx}"/>')
    d.add(f'<line class="g" x1="0" y1="{FUS_X0 - 25}" x2="0" y2="{FIN_TE + 25}"/>')
    for sg in (-1, 1):
        d.add(f'<line class="g" x1="{sg * ny}" y1="{X_AXLE - R - 25}" x2="{sg * ny}" y2="{CHORD + 25}"/>')
    # airframe
    d.add(f'<rect class="w" x="{-SPAN / 2}" y="0" width="{SPAN}" height="{CHORD}"/>')
    for sg in (-1, 1):
        ya, yb = sorted((sg * AIL_Y0, sg * AIL_Y1))
        d.add(f'<rect class="o" x="{ya}" y="{X_HINGE}" width="{yb - ya}" height="{AIL_CHORD}"/>')
    d.add(f'<rect class="b" x="{-(ny + 40)}" y="{X_SPAR - 5}" width="{2 * (ny + 40)}" height="10"/>')
    d.add(f'<line class="c" x1="{-SPAN / 2}" y1="{X_SPAR}" x2="{SPAN / 2}" y2="{X_SPAR}"/>')
    d.add(f'<rect class="b" x="{-FUS_W / 2}" y="{FUS_X0}" width="{FUS_W}" height="{FUS_X1 - FUS_X0}" rx="18"/>')
    d.add(f'<rect class="b" x="{-BOOM_D / 2}" y="{BOOM_X0}" width="{BOOM_D}" height="{BOOM_X1 - BOOM_X0}"/>')
    d.add(f'<rect class="w" x="{-B_HT / 2}" y="{X_TAIL_LE}" width="{B_HT}" height="{C_TAIL}"/>')
    d.add(f'<rect class="o" x="{-B_HT / 2}" y="{X_TAIL_TE - ELEV_C}" width="{B_HT}" height="{ELEV_C}"/>')
    d.add(f'<rect class="b" x="-2.5" y="{X_FIN0}" width="5" height="{FIN_CR}"/>')
    for sg in (-1, 1):
        y = sg * ny
        d.add(f'<rect class="r" x="{y - 15}" y="{X_AXLE - 10}" width="30" height="{X_SPAR - X_AXLE + 20}"/>')
        d.add(f'<circle class="p" cx="{y}" cy="{X_AXLE}" r="{R}"/>')
        d.add(f'<line class="c" x1="{y - R - 10}" y1="{X_AXLE}" x2="{y + R + 10}" y2="{X_AXLE}"/>')
        d.add(f'<circle cx="{sg * NAC_Y_TIP}" cy="{X_AXLE}" r="{R}" fill="none" stroke="#aaa" stroke-dasharray="4 3"/>')
    d.add(f'<circle class="p" cx="0" cy="{X_REAR}" r="{R}"/>')
    d.add(f'<circle class="cg" cx="0" cy="{X_CG_TARGET}" r="5"/>')
    d.add(f'<rect class="o" x="-17" y="{xb22 - 52}" width="34" height="105"/>')
    d.add(f'<rect class="o" x="-17" y="{xb13 - 37}" width="34" height="75" stroke-dasharray="3 2"/>')
    # --- spanwise dimensions (above the nose)
    yT1, yT2 = -265, -300
    pts = [-SPAN / 2, -ny, 0, ny, SPAN / 2]
    d.hchain(pts, yT1, [0, X_AXLE - R, FUS_X0, X_AXLE - R, 0])
    d.hdim(-SPAN / 2, SPAN / 2, yT2, f"{_n(SPAN)} span", 0, 0)
    d.hdim(-ny - R, -ny + R, X_AXLE - R - 22, f"Ø{_n(PROP_D)} (5 in)", X_AXLE, X_AXLE)
    d.hdim(ny - R, ny + R, X_AXLE - R - 22, f"Ø{_n(PROP_D)} (5 in)", X_AXLE, X_AXLE)
    # --- aileron chain below the wing TE
    yA = CHORD + 36
    apts = [-SPAN / 2, -AIL_Y1, -AIL_Y0, AIL_Y0, AIL_Y1, SPAN / 2]
    d.hchain(apts, yA, [CHORD] * 6)
    # tip-end labels outside for narrow segments
    # --- fuselage width, rear prop
    d.hdim(-FUS_W / 2, FUS_W / 2, 125, e0=None, e1=None, tp="r")
    d.hdim(-R, R, X_REAR + R + 28, f"Ø{_n(PROP_D)} (5 in)", X_REAR, X_REAR)
    # --- stabilizer span
    d.hdim(-B_HT / 2, B_HT / 2, X_TAIL_TE + 45, e0=X_TAIL_TE, e1=X_TAIL_TE)
    # --- stabilizer / elevator / fin chord (left of the stabilizer)
    d.vchain([X_TAIL_LE, X_TAIL_TE - ELEV_C, X_TAIL_TE], B_HT / 2 + 28, [B_HT / 2] * 3)
    d.vdim(-B_HT / 2 - 28, X_TAIL_LE, X_TAIL_TE, f"{_n(C_TAIL)} stab", -B_HT / 2, -B_HT / 2)
    d.vdim(-30, X_FIN0, FIN_TE, f"{_n(FIN_CR)} fin root", -2.5, -2.5)
    # --- right-hand chordwise columns
    xr1, xr2 = 755, 800
    st = 725
    d.vchain([X_AXLE, 0, CHORD, X_REAR, X_TAIL_LE, X_TAIL_TE], xr1, [st] * 6)
    d.vdim(xr2, FUS_X0, FIN_TE, f"{_n(FIN_TE - FUS_X0)} overall", st, st)
    # --- left-hand datum chain from the wing LE
    xl = -755
    d.vchain([0, X_SPAR, X_CG_TARGET, X_HINGE, CHORD], xl, [-st] * 5)
    # labels
    for (x, y, tx) in [
        (ny + R + 12, X_AXLE - 6, f"tilt axle x={_n(X_AXLE)}, y=±{_n(ny)}"),
        (-SPAN / 2 + 25, 100, "WING 1300 × 200, foam 5 mm"),
        (-SPAN / 2 + 25, X_HINGE + 32, "aileron, hinge x=150"),
        (FUS_W / 2 + 8, xb22 + 4, "batt 2200 / 1300 (dashed)"),
        (R + 12, X_REAR + 4, f"rear motor x={_n(X_REAR)}"),
        (12, X_CG_TARGET + 22, f"CG x={_n(X_CG_TARGET)}"),
        (B_HT / 2 + 80, X_TAIL_LE + 62, "STABILIZER + ELEVATOR"),
        (NAC_Y_TIP - 280, X_AXLE - R - 70, "grey dashed: wing-tip position (not recommended)"),
    ]:
        d.text(x, y, tx, "t", "start")
    d.frame(vb, "CITRUS TILTROTOR - Y tilt-tri", "TOP VIEW (nose up)", (notes0, notes1))
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vb[0]} {vb[1]} {vb[2]} {vb[3]}" '
           f'width="{vb[2]}" height="{vb[3]}">', SVG_STYLE] + d.s + ["</svg>"]
    with open(os.path.join(out, "geometry-top.svg"), "w") as f:
        f.write("\n".join(svg))

    # =============== SIDE VIEW: svg X = x, svg Y = -z ===============
    d = _Dwg()
    vb = (-440, -330, 1300, 700)
    d.add(f'<rect x="{vb[0]}" y="{vb[1]}" width="{vb[2]}" height="{vb[3]}" fill="#fff"/>')
    for gx in (X_AXLE, 0, X_SPAR, X_CG_TARGET, CHORD, X_REAR, X_TAIL_LE, X_TAIL_TE, GEAR_X_FRONT):
        d.add(f'<line class="g" x1="{gx}" y1="{-fin_top - 15}" x2="{gx}" y2="{-GEAR_Z + 15}"/>')
    d.add(f'<line x1="{vb[0] + 20}" y1="{-GEAR_Z}" x2="{vb[0] + vb[2] - 20}" y2="{-GEAR_Z}" stroke="#6a4" stroke-width="2"/>')
    d.add(f'<rect class="b" x="{FUS_X0}" y="{-FUS_ZTOP}" width="{FUS_X1 - FUS_X0}" height="{FUS_H}" rx="14"/>')
    d.add(f'<rect class="w" x="0" y="{-FOAM_T / 2}" width="{CHORD}" height="{FOAM_T}"/>')
    d.add(f'<circle class="b" cx="{X_SPAR}" cy="7.5" r="5"/>')
    d.add(f'<rect class="b" x="{BOOM_X0}" y="{-Z_BOOM - BOOM_D / 2}" width="{BOOM_X1 - BOOM_X0}" height="{BOOM_D}"/>')
    d.add(f'<rect class="r" x="{X_AXLE - 8}" y="2.5" width="{X_SPAR - X_AXLE + 20}" height="14"/>')
    for th, dash in ((0, ""), (90, ""), (-15, ' stroke-dasharray="5 3"')):
        (ax, az), (bx, bz) = disc_pts(th, 2)
        d.add(f'<line x1="{ax:.1f}" y1="{-az:.1f}" x2="{bx:.1f}" y2="{-bz:.1f}" stroke="#2a62b8" stroke-width="3"{dash}/>')
    d.add(f'<circle cx="{X_AXLE}" cy="{-Z_AXLE}" r="{math.hypot(H_PROP, R):.1f}" class="c"/>')
    d.add(f'<circle cx="{X_AXLE}" cy="{-Z_AXLE}" r="3" fill="#b33"/>')
    d.add(f'<line x1="{X_REAR - R}" y1="{-zr}" x2="{X_REAR + R}" y2="{-zr}" stroke="#2a62b8" stroke-width="3"/>')
    d.add(f'<rect class="b" x="{X_REAR - 14}" y="{-zr}" width="28" height="{zr - Z_BOOM - BOOM_D / 2}"/>')
    d.add(f'<rect class="w" x="{X_TAIL_LE}" y="{-zt}" width="{C_TAIL}" height="{FOAM_T}"/>')
    d.add(f'<polygon class="w" points="{X_FIN0},{-zt} {FIN_TE},{-zt} {FIN_TE},{-fin_top} {FIN_TE - FIN_CT},{-fin_top}"/>')
    d.add(f'<line class="c" x1="{FIN_TE - RUDDER_C}" y1="{-(zt + RUDDER_GAP)}" x2="{FIN_TE - RUDDER_C}" y2="{-fin_top}"/>')
    for gx in (GEAR_X_FRONT, GEAR_X_REAR):
        top = -12 if gx == GEAR_X_FRONT else Z_BOOM - BOOM_D / 2
        d.add(f'<line x1="{gx}" y1="{-top}" x2="{gx}" y2="{-GEAR_Z}" stroke="#555" stroke-width="4"/>')
    d.add(f'<circle class="cg" cx="{X_CG_TARGET}" cy="30" r="5"/>')
    # tilt angle callout (hover = thrust up, cruise = thrust forward)
    rA = 30
    ax_, ay_ = X_AXLE, -Z_AXLE
    d.add(f'<path class="d" d="M {ax_} {ay_ - rA} A {rA} {rA} 0 0 0 {ax_ - rA} {ay_}"/>')
    d.arrow(ax_, ay_ - rA, 1, 0)
    d.arrow(ax_ - rA, ay_, 0, 1)
    d.text(ax_ - rA - 4, ay_ - rA + 2, "90° tilt", "dt", "end")
    # --- chordwise dimensions above the airframe
    yU = -90
    d.hchain([X_AXLE, 0, X_SPAR, X_CG_TARGET, X_HINGE, CHORD], yU,
             [-Z_AXLE, 0, 0, 30, 0, 0])
    d.hdim(X_REAR - R, X_REAR + R, -zr - 24, f"Ø{_n(PROP_D)}", -zr, -zr)
    d.hdim(FIN_TE - FIN_CT, FIN_TE, -fin_top - 22, f"{_n(FIN_CT)} fin tip", -fin_top, -fin_top)
    d.hdim(FIN_TE - RUDDER_C, FIN_TE, -fin_top - 52, f"{_n(RUDDER_C)} rudder (opt.)", -fin_top, -fin_top)
    # --- stations below the ground line
    y1, y2, y3 = -GEAR_Z + 40, -GEAR_Z + 80, -GEAR_Z + 120
    sx = [FUS_X0, X_AXLE, GEAR_X_FRONT, 0, CHORD, X_REAR, X_TAIL_LE, X_TAIL_TE]
    d.hchain(sx, y1, [-GEAR_Z] * len(sx))
    d.hdim(FUS_X0, FIN_TE, y2, f"{_n(FIN_TE - FUS_X0)} overall (nose to fin TE)", -GEAR_Z, -GEAR_Z)
    d.hdim(GEAR_X_FRONT, GEAR_X_REAR, y3, f"{_n(GEAR_X_REAR - GEAR_X_FRONT)} gear base", -GEAR_Z, -GEAR_Z)
    # --- heights, left column
    xl1, xl2 = FUS_X0 - 35, FUS_X0 - 75
    h_up = Z_AXLE + H_PROP
    d.vchain([-GEAR_Z, -(FUS_ZTOP - FUS_H), -Z_AXLE, -h_up], xl1,
             [FUS_X0 + 10, FUS_X0, X_AXLE - 6, X_AXLE - R - 6])
    d.vdim(xl2, -GEAR_Z, -(Z_AXLE + H_PROP), None, X_AXLE - R - 6, X_AXLE - R - 6)
    d.vdim(xl2 - 40, -GEAR_Z, 0, f"{_n(-GEAR_Z)} ground to wing", FUS_X0, FUS_X0)
    # --- heights, right column
    xr1, xr2 = FIN_TE + 40, FIN_TE + 80
    d.vchain([-GEAR_Z, -zt, -fin_top], xr1, [FIN_TE] * 3)
    d.vdim(xr2, -GEAR_Z, -fin_top, f"{_n(fin_top - GEAR_Z)} ground to fin top", FIN_TE, FIN_TE)
    d.vdim(X_REAR + R + 45, -zr, -(Z_BOOM + BOOM_D / 2), f"{_n(zr - Z_BOOM - BOOM_D / 2)}",
           X_REAR + R, X_REAR + R)
    # labels
    for (x, y, tx) in [
        (X_AXLE - 20, -(Z_AXLE + H_PROP) - 12, "hover disc (0°); dashed: -15°"),
        (X_AXLE - H_PROP - 120, -Z_AXLE + 70, "cruise disc (90°)"),
        (X_AXLE - 30, -Z_AXLE + 38, f"axle x={_n(X_AXLE)} z={_n(Z_AXLE)}"),
        (X_CG_TARGET + 8, 44, "CG"),
        (X_TAIL_LE - 10, -fin_top + 10, "fin"),
    ]:
        d.text(x, y, tx, "t", "start")
    d.frame(vb, "CITRUS TILTROTOR - Y tilt-tri", f"SIDE VIEW (nose left, ground z={_n(GEAR_Z)})", (notes0, notes1))
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vb[0]} {vb[1]} {vb[2]} {vb[3]}" '
           f'width="{vb[2]}" height="{vb[3]}">', SVG_STYLE] + d.s + ["</svg>"]
    with open(os.path.join(out, "geometry-side.svg"), "w") as f:
        f.write("\n".join(svg))
    print("\nWrote docs/geometry-top.svg and docs/geometry-side.svg")


if __name__ == "__main__":
    main()
