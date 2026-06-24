"""Pure PyBullet AUV physics model (no ROS dependencies).

A single rigid cuboid (the AUV) in a hollow box of "water". Buoyancy + per-axis
drag stand in for being submerged -- there is no fluid solver. Forces are applied
as a single whole-body wrench (no per-thruster geometry).

This module can be exercised standalone (see __main__) for a headless smoke test.

Frames: world = ENU (+Z up), body = REP-103 (+X forward, +Y left, +Z up).
Water surface at z = water_surface_z (default 0). Depth below surface is +ve.
"""

import math
from dataclasses import dataclass, field
from typing import List

import pybullet as p
import pybullet_data


# Named RGBA colors for config-by-name props (e.g. drums).
_COLORS = {
    "red": [0.9, 0.1, 0.1, 1.0],
    "blue": [0.1, 0.3, 0.9, 1.0],
    "green": [0.1, 0.8, 0.1, 1.0],
    "orange": [1.0, 0.5, 0.0, 1.0],
    "yellow": [0.9, 0.8, 0.1, 1.0],
    "white": [1.0, 1.0, 1.0, 1.0],
    "black": [0.05, 0.05, 0.05, 1.0],
}


@dataclass
class AuvConfig:
    """All tunable physics parameters. Defaults are BlueROV2-scale and documented
    in config/sim_params.yaml -- keep the two in sync."""

    # Body
    mass: float = 11.0                                   # kg
    half_extents: List[float] = field(default_factory=lambda: [0.30, 0.20, 0.20])  # m (0.60 x 0.40 x 0.40)

    # Environment
    gravity: float = 9.81                                # m/s^2
    water_density: float = 1000.0                        # kg/m^3 (freshwater)
    water_surface_z: float = 0.0                         # m (world up = +z)

    # Buoyancy
    displaced_volume: float = 0.01155                    # m^3 -> slightly positive
    buoyancy_ramp_height: float = 0.05                   # m, submerged-fraction ramp band
    cob_offset_body: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.02])  # m, above CoG

    # Hydrodynamic drag: F = -c_lin*v - c_quad*|v|*v, per axis.
    drag_linear: List[float] = field(default_factory=lambda: [40.0, 60.0, 80.0])    # N/(m/s)
    drag_quadratic: List[float] = field(default_factory=lambda: [60.0, 90.0, 120.0])  # N/(m/s)^2
    angular_drag_linear: List[float] = field(default_factory=lambda: [4.0, 4.0, 3.0])     # Nm/(rad/s)
    angular_drag_quadratic: List[float] = field(default_factory=lambda: [6.0, 6.0, 4.0])  # Nm/(rad/s)^2

    # Max whole-body wrench (normalized command of 1.0 -> these)
    max_surge_n: float = 100.0
    max_sway_n: float = 80.0
    max_heave_n: float = 100.0
    max_roll_nm: float = 5.0
    max_pitch_nm: float = 5.0
    max_yaw_nm: float = 8.0

    # Pool (inner usable volume), centered at origin in x/y; top at z=0.
    pool_length: float = 25.0    # x, m (long)
    pool_width: float = 16.0     # y, m (wide)
    pool_depth: float = 2.0      # z, m (floor at -pool_depth)
    pool_wall_thickness: float = 0.2

    # Static square gate ('ring') the AUV can swim through.
    ring_enabled: bool = True
    ring_size: float = 1.40            # outer side length (m)
    ring_thickness: float = 0.02       # bar cross-section (m), square
    ring_pos: List[float] = field(default_factory=lambda: [0.0, 3.0, -1.0])  # world center
    ring_rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # orientation (rad)

    # Static orange post (vertical cuboid marker).
    post_enabled: bool = True
    post_height: float = 1.60          # m, along local Z
    post_thickness: float = 0.15       # m, square cross-section
    post_pos: List[float] = field(default_factory=lambda: [2.0, 0.0, -1.0])  # world center
    post_rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # orientation (rad)

    # Static inverted-U gate (two legs + top bar; legs stand on the pool floor).
    gate_enabled: bool = True
    gate_width: float = 1.50           # m, top-bar outer span (leg to leg)
    gate_height: float = 1.00          # m, leg height (opening height under the bar)
    gate_thickness: float = 0.15       # m, square bar cross-section
    # gate_pos is the base center on the floor; z should be -pool_depth to stand on it.
    gate_pos: List[float] = field(default_factory=lambda: [-3.0, 0.0, -2.0])
    gate_rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # orientation (rad)

    # Static drums (upright cylinders resting on the floor). One per (x,y,z) triple
    # in drum_positions; drum_colors names them by index (extra drums default red).
    drums_enabled: bool = True
    drum_diameter: float = 0.60        # m
    drum_depth: float = 0.30           # m (cylinder height, along Z)
    drum_positions: List[float] = field(default_factory=lambda: [
        3.0, 0.0, -1.85,
        4.0, 0.0, -1.85,
        3.0, 1.0, -1.85,
        4.0, 1.0, -1.85,
    ])
    drum_colors: List[str] = field(default_factory=lambda: ["blue", "red", "red", "red"])

    # Static flares (foot + thin pole on the floor) each topped by a FREE-moving
    # (dynamic) golf ball the AUV can knock off. One flare per (x,y,z) triple;
    # flare_colors names the pole by index. flare position z should be -pool_depth.
    flares_enabled: bool = True
    flare_height: float = 0.80         # m, pole height
    flare_pole_diameter: float = 0.03  # m
    flare_base_diameter: float = 0.12  # m, foot disk
    flare_base_height: float = 0.03    # m, foot thickness
    flare_ball_diameter: float = 0.043  # m, golf ball
    flare_ball_mass: float = 0.046     # kg, golf ball (dynamic)
    flare_positions: List[float] = field(default_factory=lambda: [
        -6.0, -1.5, -2.0,
        -6.0, 0.0, -2.0,
        -6.0, 1.5, -2.0,
    ])
    flare_colors: List[str] = field(default_factory=lambda: ["red", "yellow", "blue"])

    # Water surface visual plane (ultra-thin flat cuboid at z=water_surface_z).
    surface_plane_enabled: bool = True
    surface_plane_thickness: float = 0.0001  # m, 0.1 mm ultra-thin

    # Integration
    timestep: float = 1.0 / 240.0

    # Initial spawn pose. Slightly submerged so buoyancy lifts it to the surface.
    # start_yaw is the heading about world +Z (rad). pi/2 makes the vehicle face
    # world +Y, so: forward->+Y, backward->-Y, lateral-right->+X, lateral-left->-X.
    start_x: float = 0.0
    start_y: float = 0.0
    start_z: float = -0.5
    start_yaw: float = math.pi / 2.0


@dataclass
class Command:
    """Latest normalized 6-DOF command, each in [-1, 1], plus arm state."""
    arm: bool = False
    surge: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


class AuvSim:
    """Owns the PyBullet world and the AUV body, applies the wrench model."""

    def __init__(self, cfg: AuvConfig, gui: bool = False):
        self.cfg = cfg
        self.gui = gui
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        if gui:
            # Strip GUI clutter we don't use: the Explorer/Params side panels and the
            # three Synthetic Camera (RGB/Depth/Segmentation) preview boxes. (The fixed
            # floor reference grid is baked into the viewer and cannot be toggled off.)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0, physicsClientId=self.client)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0, physicsClientId=self.client)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0, physicsClientId=self.client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(0, 0, -cfg.gravity, physicsClientId=self.client)
        p.setTimeStep(cfg.timestep, physicsClientId=self.client)
        # We step explicitly.
        p.setRealTimeSimulation(0, physicsClientId=self.client)

        self._build_pool()
        if cfg.surface_plane_enabled:
            self._build_surface_plane()
        if cfg.ring_enabled:
            self._build_ring()
        if cfg.post_enabled:
            self._build_post()
        if cfg.gate_enabled:
            self._build_gate()
        if cfg.drums_enabled:
            self._build_drums()
        if cfg.flares_enabled:
            self._build_flares()
        self.body = self._build_auv()
        self.sim_time = 0.0

    # -- world construction ------------------------------------------------
    def _build_pool(self):
        cfg = self.cfg
        # Vehicle forward is +Y, so the long axis (pool_length) runs along Y and
        # the width (pool_width) along X (left/right).
        half_y = cfg.pool_length / 2.0   # forward/back span (long, +Y)
        half_x = cfg.pool_width / 2.0    # left/right span   (short, +X)
        d = cfg.pool_depth
        t = cfg.pool_wall_thickness
        surface = cfg.water_surface_z

        def static_box(half_extents, pos):
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents,
                                         physicsClientId=self.client)
            vis = -1
            if self.gui:
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                          rgbaColor=[0.6, 0.8, 1.0, 1.0],  # light blue pool
                                          physicsClientId=self.client)
            return p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col,
                                     baseVisualShapeIndex=vis,
                                     basePosition=pos, physicsClientId=self.client)

        floor_z = surface - d
        # Floor
        static_box([half_x, half_y, t / 2.0], [0, 0, floor_z - t / 2.0])
        # Walls (run from floor up to the surface)
        wall_h = d
        wall_cz = surface - d / 2.0
        # Left/right walls (normal X), span the full forward length and depth
        static_box([t / 2.0, half_y, wall_h / 2.0], [half_x + t / 2.0, 0, wall_cz])   # +x
        static_box([t / 2.0, half_y, wall_h / 2.0], [-half_x - t / 2.0, 0, wall_cz])  # -x
        # Front/back walls (normal Y), span the full width and depth
        static_box([half_x, t / 2.0, wall_h / 2.0], [0, half_y + t / 2.0, wall_cz])   # +y
        static_box([half_x, t / 2.0, wall_h / 2.0], [0, -half_y - t / 2.0, wall_cz])  # -y

    def _build_surface_plane(self):
        """Ultra-thin flat visual plane at the water surface (z = water_surface_z)
        to visualize where the water surface is. No collision (visual only)."""
        cfg = self.cfg
        half_y = cfg.pool_length / 2.0
        half_x = cfg.pool_width / 2.0
        h = cfg.surface_plane_thickness / 2.0  # half thickness
        vis = -1
        if self.gui:
            vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[half_x - 0.1, half_y - 0.1, h],  # slightly inset from pool edge
                rgbaColor=[0.3, 0.7, 1.0, 0.5],  # cyan, semi-transparent
                physicsClientId=self.client)
        # Visual-only body (no collision, mass 0).
        p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=-1,
                          baseVisualShapeIndex=vis,
                          basePosition=[0.0, 0.0, cfg.water_surface_z],
                          physicsClientId=self.client)

    def _build_ring(self):
        """Static square gate ('ring') the AUV can swim through.

        Four box bars in the local X-Z plane (opening normal = local Y), so with
        default orientation the AUV passes through along its forward +Y axis.
        """
        cfg = self.cfg
        c = cfg.ring_thickness / 2.0     # half bar cross-section
        s = cfg.ring_size / 2.0          # half outer side length
        inner = s - cfg.ring_thickness   # vertical-bar half length (fits between top/bottom)

        half_extents = [
            [s, c, c],        # top bar    (spans X)
            [s, c, c],        # bottom bar (spans X)
            [c, c, inner],    # left bar   (spans Z)
            [c, c, inner],    # right bar  (spans Z)
        ]
        frames = [
            [0.0, 0.0,  s - c],
            [0.0, 0.0, -(s - c)],
            [-(s - c), 0.0, 0.0],
            [ (s - c), 0.0, 0.0],
        ]
        col = p.createCollisionShapeArray(
            shapeTypes=[p.GEOM_BOX] * 4, halfExtents=half_extents,
            collisionFramePositions=frames, physicsClientId=self.client)
        vis = -1
        if self.gui:
            vis = p.createVisualShapeArray(
                shapeTypes=[p.GEOM_BOX] * 4, halfExtents=half_extents,
                visualFramePositions=frames,
                rgbaColors=[[0.9, 0.15, 0.15, 1.0]] * 4,   # red
                physicsClientId=self.client)
        p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=list(cfg.ring_pos),
            baseOrientation=p.getQuaternionFromEuler(list(cfg.ring_rpy)),
            physicsClientId=self.client)

    def _build_post(self):
        """Static orange vertical post (cuboid marker)."""
        cfg = self.cfg
        h = cfg.post_thickness / 2.0     # half cross-section
        v = cfg.post_height / 2.0        # half height
        half_extents = [h, h, v]         # square in X-Y, tall in Z
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents,
                                     physicsClientId=self.client)
        vis = -1
        if self.gui:
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                      rgbaColor=[1.0, 0.5, 0.0, 1.0],  # orange
                                      physicsClientId=self.client)
        p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=list(cfg.post_pos),
            baseOrientation=p.getQuaternionFromEuler(list(cfg.post_rpy)),
            physicsClientId=self.client)

    def _build_gate(self):
        """Static inverted-U gate: two vertical legs + a horizontal top bar.

        Local origin is the base center on the floor; legs rise along +Z, the bar
        spans X, and the opening normal is local Y (AUV swims through along +Y with
        default orientation). Left leg red, right leg green, top bar black.
        """
        cfg = self.cfg
        c = cfg.gate_thickness / 2.0     # half bar cross-section
        hw = cfg.gate_width / 2.0        # half top-bar span
        h = cfg.gate_height              # leg height (floor -> underside of bar)
        leg_x = hw - c                   # leg center offset so outer face meets bar end

        half_extents = [
            [c, c, h / 2.0],    # left leg  (vertical)
            [c, c, h / 2.0],    # right leg (vertical)
            [hw, c, c],         # top bar   (spans X), sits on top of the legs
        ]
        frames = [
            [-leg_x, 0.0, h / 2.0],
            [ leg_x, 0.0, h / 2.0],
            [0.0, 0.0, h + c],
        ]
        col = p.createCollisionShapeArray(
            shapeTypes=[p.GEOM_BOX] * 3, halfExtents=half_extents,
            collisionFramePositions=frames, physicsClientId=self.client)
        vis = -1
        if self.gui:
            vis = p.createVisualShapeArray(
                shapeTypes=[p.GEOM_BOX] * 3, halfExtents=half_extents,
                visualFramePositions=frames,
                rgbaColors=[
                    [0.9, 0.1, 0.1, 1.0],     # left leg  red
                    [0.1, 0.8, 0.1, 1.0],     # right leg green
                    [0.05, 0.05, 0.05, 1.0],  # top bar   black
                ],
                physicsClientId=self.client)
        p.createMultiBody(
            baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=list(cfg.gate_pos),
            baseOrientation=p.getQuaternionFromEuler(list(cfg.gate_rpy)),
            physicsClientId=self.client)

    def _build_drums(self):
        """Static upright cylinders ('drums'). One per (x,y,z) in drum_positions;
        drum_colors names them by index (missing/unknown -> red)."""
        cfg = self.cfg
        r = cfg.drum_diameter / 2.0
        h = cfg.drum_depth
        pos = list(cfg.drum_positions)
        colors = list(cfg.drum_colors)
        n = len(pos) // 3
        # Same geometry for every drum -> one shared collision shape.
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h,
                                     physicsClientId=self.client)
        for i in range(n):
            center = pos[3 * i:3 * i + 3]
            vis = -1
            if self.gui:
                name = colors[i] if i < len(colors) else "red"
                rgba = _COLORS.get(name, _COLORS["red"])
                vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h,
                                          rgbaColor=rgba, physicsClientId=self.client)
            p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col,
                              baseVisualShapeIndex=vis, basePosition=center,
                              physicsClientId=self.client)

    def _build_flares(self):
        """Static flares (foot + thin pole on the floor) each topped by a FREE
        (dynamic) golf ball the AUV can knock off. Flare position is the base
        point on the floor (z should be -pool_depth)."""
        cfg = self.cfg
        pr = cfg.flare_pole_diameter / 2.0
        ph = cfg.flare_height
        br = cfg.flare_base_diameter / 2.0
        bh = cfg.flare_base_height
        ball_r = cfg.flare_ball_diameter / 2.0
        pos = list(cfg.flare_positions)
        colors = list(cfg.flare_colors)
        n = len(pos) // 3

        foot_z = bh / 2.0            # foot center above the floor
        pole_z = bh + ph / 2.0       # pole center
        ball_z = bh + ph + ball_r    # ball rests on the pole top

        # Shared static stand (foot + pole) and ball collision shapes.
        stand_col = p.createCollisionShapeArray(
            shapeTypes=[p.GEOM_CYLINDER, p.GEOM_CYLINDER],
            radii=[br, pr], lengths=[bh, ph],
            collisionFramePositions=[[0, 0, foot_z], [0, 0, pole_z]],
            physicsClientId=self.client)
        ball_col = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_r,
                                          physicsClientId=self.client)
        for i in range(n):
            base = pos[3 * i:3 * i + 3]
            name = colors[i] if i < len(colors) else "red"
            rgba = _COLORS.get(name, _COLORS["red"])
            stand_vis = ball_vis = -1
            if self.gui:
                stand_vis = p.createVisualShapeArray(
                    shapeTypes=[p.GEOM_CYLINDER, p.GEOM_CYLINDER],
                    radii=[br, pr], lengths=[bh, ph],
                    visualFramePositions=[[0, 0, foot_z], [0, 0, pole_z]],
                    rgbaColors=[rgba, rgba], physicsClientId=self.client)
                ball_vis = p.createVisualShape(
                    p.GEOM_SPHERE, radius=ball_r, rgbaColor=[0.9, 0.9, 0.9, 1.0],
                    physicsClientId=self.client)
            # Static stand.
            p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=stand_col,
                              baseVisualShapeIndex=stand_vis, basePosition=base,
                              physicsClientId=self.client)
            # Free-moving golf ball balanced on top.
            p.createMultiBody(
                baseMass=cfg.flare_ball_mass, baseCollisionShapeIndex=ball_col,
                baseVisualShapeIndex=ball_vis,
                basePosition=[base[0], base[1], base[2] + ball_z],
                physicsClientId=self.client)

    def _build_auv(self):
        cfg = self.cfg
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=cfg.half_extents,
                                     physicsClientId=self.client)
        vis = -1
        if self.gui:
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=cfg.half_extents,
                                      rgbaColor=[0.9, 0.7, 0.1, 1.0],
                                      physicsClientId=self.client)
        start_orn = p.getQuaternionFromEuler([0.0, 0.0, cfg.start_yaw])
        body = p.createMultiBody(baseMass=cfg.mass, baseCollisionShapeIndex=col,
                                 baseVisualShapeIndex=vis,
                                 basePosition=[cfg.start_x, cfg.start_y, cfg.start_z],
                                 baseOrientation=start_orn,
                                 physicsClientId=self.client)
        # A little linear/angular sleep damping disabled -- we model drag ourselves.
        p.changeDynamics(body, -1, linearDamping=0.0, angularDamping=0.0,
                         physicsClientId=self.client)
        return body

    # -- per-step force model ----------------------------------------------
    def _apply_command_wrench(self, cmd: Command):
        if not cmd.arm:
            return
        cfg = self.cfg
        # Body-frame force / torque from normalized commands.
        fx = cmd.surge * cfg.max_surge_n
        fy = cmd.sway * cfg.max_sway_n     # +Y = sway-left (single, consistent sign)
        fz = cmd.heave * cfg.max_heave_n
        tx = cmd.roll * cfg.max_roll_nm
        ty = cmd.pitch * cfg.max_pitch_nm
        tz = cmd.yaw * cfg.max_yaw_nm

        # Force at CoG in the link (body) frame.
        p.applyExternalForce(self.body, -1, [fx, fy, fz], [0, 0, 0],
                             p.LINK_FRAME, physicsClientId=self.client)
        # Torque: rotate body-frame torque into world frame for reliable behavior.
        _, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self.client)
        tw = self._rotate_body_to_world([tx, ty, tz], orn)
        p.applyExternalTorque(self.body, -1, tw, p.WORLD_FRAME,
                              physicsClientId=self.client)

    def _apply_buoyancy(self):
        cfg = self.cfg
        pos, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self.client)

        # Submerged fraction: ramp 0->1 as the body crosses the surface over the
        # ramp band, so the float settles smoothly instead of bobbing.
        top_of_band = cfg.water_surface_z
        frac = (top_of_band - pos[2]) / max(cfg.buoyancy_ramp_height, 1e-6)
        frac = max(0.0, min(1.0, frac))
        if frac <= 0.0:
            return

        f_buoy = cfg.water_density * cfg.gravity * cfg.displaced_volume * frac
        # Apply at the center of buoyancy (offset above CoG in body frame) so the
        # body passively self-rights in roll/pitch.
        cob_world_offset = self._rotate_body_to_world(cfg.cob_offset_body, orn)
        cob_pos = [pos[0] + cob_world_offset[0],
                   pos[1] + cob_world_offset[1],
                   pos[2] + cob_world_offset[2]]
        p.applyExternalForce(self.body, -1, [0, 0, f_buoy], cob_pos,
                             p.WORLD_FRAME, physicsClientId=self.client)

    def _apply_drag(self):
        cfg = self.cfg
        lin_w, ang_w = p.getBaseVelocity(self.body, physicsClientId=self.client)
        _, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self.client)

        # Linear drag in body frame.
        lin_b = self._rotate_world_to_body(lin_w, orn)
        fb = [-(cfg.drag_linear[i] * lin_b[i]
                + cfg.drag_quadratic[i] * abs(lin_b[i]) * lin_b[i]) for i in range(3)]
        p.applyExternalForce(self.body, -1, fb, [0, 0, 0], p.LINK_FRAME,
                             physicsClientId=self.client)

        # Angular drag in body frame.
        ang_b = self._rotate_world_to_body(ang_w, orn)
        tb = [-(cfg.angular_drag_linear[i] * ang_b[i]
                + cfg.angular_drag_quadratic[i] * abs(ang_b[i]) * ang_b[i]) for i in range(3)]
        tw = self._rotate_body_to_world(tb, orn)
        p.applyExternalTorque(self.body, -1, tw, p.WORLD_FRAME,
                              physicsClientId=self.client)

    def step(self, cmd: Command):
        """Advance one fixed physics timestep applying the current command."""
        self._apply_command_wrench(cmd)
        self._apply_buoyancy()
        self._apply_drag()
        p.stepSimulation(physicsClientId=self.client)
        self.sim_time += self.cfg.timestep

    # -- state readout ------------------------------------------------------
    def get_state(self):
        """Ground-truth state dict in the internal ENU/REP-103 frames."""
        pos, orn = p.getBasePositionAndOrientation(self.body, physicsClientId=self.client)
        lin_w, ang_w = p.getBaseVelocity(self.body, physicsClientId=self.client)
        ang_b = self._rotate_world_to_body(ang_w, orn)
        lin_b = self._rotate_world_to_body(lin_w, orn)
        return {
            "sim_time": self.sim_time,
            "position": list(pos),          # world ENU
            "orientation": list(orn),       # quaternion (x, y, z, w)
            "linear_velocity": list(lin_w),  # world ENU
            "linear_velocity_body": list(lin_b),
            "angular_velocity_world": list(ang_w),
            "angular_velocity_body": list(ang_b),
            "depth": max(0.0, self.cfg.water_surface_z - pos[2]),
        }

    # -- helpers ------------------------------------------------------------
    def _rotate_body_to_world(self, v, orn):
        m = p.getMatrixFromQuaternion(orn)
        return [m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
                m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
                m[6] * v[0] + m[7] * v[1] + m[8] * v[2]]

    def _rotate_world_to_body(self, v, orn):
        m = p.getMatrixFromQuaternion(orn)  # row-major R (body->world)
        # world->body uses R^T
        return [m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
                m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
                m[2] * v[0] + m[5] * v[1] + m[8] * v[2]]

    def close(self):
        p.disconnect(physicsClientId=self.client)


def _smoke_test():
    """Headless standalone check: float to surface, then surge forward when armed."""
    cfg = AuvConfig()
    sim = AuvSim(cfg, gui=False)
    print("Settling (neutral, disarmed)...")
    cmd = Command()
    for _ in range(int(3.0 / cfg.timestep)):
        sim.step(cmd)
    s = sim.get_state()
    print(f"  z={s['position'][2]:+.3f} m  depth={s['depth']:.3f} m  "
          f"(expect near surface, slightly below 0)")

    print("Surging (armed, forward=1.0) for 2 s...")
    x0 = sim.get_state()["position"][0]
    cmd = Command(arm=True, surge=1.0)
    for _ in range(int(2.0 / cfg.timestep)):
        sim.step(cmd)
    x1 = sim.get_state()["position"][0]
    print(f"  x: {x0:+.3f} -> {x1:+.3f} m  (expect increase)")

    print("Disarmed, should coast to rest...")
    cmd = Command(arm=False)
    for _ in range(int(3.0 / cfg.timestep)):
        sim.step(cmd)
    v = sim.get_state()["linear_velocity"]
    print(f"  speed={ (v[0]**2+v[1]**2+v[2]**2) ** 0.5 :.3f} m/s  (expect small)")
    sim.close()
    print("OK")


if __name__ == "__main__":
    _smoke_test()
