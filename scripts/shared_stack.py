"""Isaac plant/cameras for the robotics_lab controller. No Python IK/follower.

The scene builder calls run_episode after creating articulations and sensors,
before instantiating any legacy policy or control object.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import pathlib
import sys
import time
from dataclasses import replace

import numpy as np


def run_episode(args, settings, scene, world, rep, arts, cameras, tcp_views,
                bolt_views, bolt_prims, overview, materials):
    root = (scene.ROOT / settings["robotics_lab"]).resolve()
    sys.path.insert(0, str(root / "policy_runner"))
    from policy_runner.isaac_transport import (
        SimulationClock, ServoBridge, PipeDatagram, BridgeStateClient,
        TimedInferenceClient, BoundedSource,
    )
    from policy_runner.camera_bundle_client import CameraBundle, CameraFrame
    from policy_runner.chunk_overlay_publisher import ChunkOverlayPublisher
    from policy_runner.config import load_config
    from policy_runner.main import run
    from policy_runner.servo_command_client import ServoCommandClient
    from isaacsim.core.utils.types import ArticulationAction
    from scipy.spatial.transform import Rotation
    from PIL import Image
    from pxr import Usd, UsdGeom, UsdShade

    if settings["schema"] != "robotics_lab.isaac_shared_stack.v1":
        raise ValueError("unsupported shared-stack settings")
    if settings["physics_dt_sec"] != 0.002 or world.get_physics_dt() != 0.002:
        raise ValueError("C++ and PhysX must both use 2 ms")
    if scene.SIM_ARM != "rb5_850e":
        raise ValueError("shared-stack profile requires the RB5 assets")
    for key in ("arm_stiffness", "arm_damping", "gripper_stiffness",
                "gripper_damping", "gripper_max_force_n"):
        if not math.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError(f"invalid PhysX drive parameter: {key}")

    out = scene.ROOT / "outputs/shared_stack" / args.tag
    out.mkdir(parents=True, exist_ok=False)
    clock = SimulationClock()
    arm_indices, finger_indices = {}, {}
    for side, art in arts.items():
        names = list(art.dof_names)
        arm_indices[side] = np.array([names.index(j) for j in scene.ARM_JOINTS])
        finger_indices[side] = np.array([names.index(j) for j in
                                       ("finger_left_joint", "finger_right_joint")])
        q = np.zeros(len(names), dtype=np.float32)
        q[arm_indices[side]] = np.deg2rad(scene.RESET[side])
        # Position assignment is RESET ONLY. Every subsequent move is a drive.
        art.set_joint_positions(q)
        art.set_joint_velocities(np.zeros_like(q))
        kp = np.full(len(names), settings["arm_stiffness"])
        kd = np.full(len(names), settings["arm_damping"])
        kp[finger_indices[side]] = settings["gripper_stiffness"]
        kd[finger_indices[side]] = settings["gripper_damping"]
        art.get_articulation_controller().set_gains(kps=kp, kds=kd)
        art.apply_action(ArticulationAction(joint_positions=q))

    poses = scene.bolt_poses(args.layout, args.n_per_color, args.seed)
    frozen = scene._SCENE_STATES.get(str(args.seed)) if args.scene_states else None
    if frozen:
        scene.work_surface.check_frozen(frozen, scene.WORK_SURFACE)
    if args.scene_states and (frozen is None or len(frozen["bolts"]) != len(bolt_prims)):
        raise ValueError("frozen scene seed/count mismatch")
    if len(poses) != len(bolt_prims):
        raise ValueError("generated scene bolt count mismatch")
    colors = list(frozen["colors"]) if frozen else [color for color, *_ in poses]
    color_counts = {color: colors.count(color) for color in ("gray", "black")}
    if (len(colors) != len(bolt_prims) or
            any(count != args.n_per_color for count in color_counts.values())):
        raise ValueError(f"scene must contain {args.n_per_color} bolts of each color: {color_counts}")
    visual_bindings = []
    for i, (path, (_, x, y, yaw)) in enumerate(zip(bolt_prims, poses)):
        start_z = scene.PICK_SURFACE_Z + (scene.bolt_max_radius() + 0.001 if 'size_m' in scene.WORK_SURFACE
                                          else scene.SHAFT_R + 0.002)
        pos = frozen["bolts"][i]["p"] if frozen else [x, y, start_z]
        quat = frozen["bolts"][i]["q"] if frozen else [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
        bolt_views[path].set_world_poses(np.array([pos]), np.array([quat]))
        bolt_views[path].set_velocities(np.zeros((1, 6)))
        # Scene construction only adds geometry/physics. The legacy episode
        # reset assigns visual materials; this independent reset must too.
        # Reuse those exact materials, keeping physics-purpose bindings intact.
        material = materials["bolt_" + colors[i]]
        built = world.stage.GetPrimAtPath(path).GetAttribute("simulation:boltColor")
        if built and built.Get() != colors[i]:
            # EVAL_BOLT_GEOMETRY binds a head shape to a bolt index at build time
            raise RuntimeError(f"{path} was built as a {built.Get()} bolt but this scene makes it {colors[i]}")
        parts = ["shaft", "head"] + [part for part in ("thread", "head_visual")
                                      if world.stage.GetPrimAtPath(f"{path}/{part}").IsValid()]
        for part in parts:
            prim = world.stage.GetPrimAtPath(f"{path}/{part}")
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            binding.Bind(material)
            bound, _ = binding.ComputeBoundMaterial()
            if not bound or bound.GetPath() != material.GetPath():
                raise RuntimeError(f"bolt visual material did not bind: {path}/{part}")
            visual_bindings.append({"prim": str(prim.GetPath()), "color": colors[i],
                                    "material": str(bound.GetPath())})
    scene_metadata = {"layout": args.layout, "seed": args.seed,
                      "work_surface": scene.WORK_SURFACE,
                      "work_surface_verification": scene.work_surface.verify_stage(
                          world.stage, scene.WORK_SURFACE, scene.TABLE_Z),
                      "pick_surface_z_m": scene.PICK_SURFACE_Z,
                      "bolt_colors": colors, "bolt_color_counts": color_counts,
                      "bolt_geometry": scene.bolt_geometry_metadata(),
                      "bolt_proxies": [scene.bolt_aabb_proxies(c) for c in colors],
                      "visual_bindings": visual_bindings}
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render','proxy'])
    tray_geometry = {}
    for name, color in (('box_gray','gray'), ('box_green','black')):
        extent = bounds.ComputeWorldBound(world.stage.GetPrimAtPath('/World/scene/'+name)).ComputeAlignedRange()
        lo, hi = np.array(extent.GetMin()), np.array(extent.GetMax())
        expected_lo = [scene.BOX_X-scene.BOX['hw'],scene.BOX_CY[color]-scene.BOX['hd'],scene.TABLE_Z]
        expected_hi = [scene.BOX_X+scene.BOX['hw'],scene.BOX_CY[color]+scene.BOX['hd'],scene.TABLE_Z+2*scene.BOX['wall_h']]
        if not np.allclose([lo,hi],[expected_lo,expected_hi],atol=1e-6,rtol=0):
            raise RuntimeError(f'{name}: tray bounds differ from measured outer dimensions: {lo}, {hi}')
        tray_geometry[name] = dict(min_m=lo.tolist(),max_m=hi.tolist(),size_m=(hi-lo).tolist(),
                                   wall_thickness_m=scene.BOX['t'])
    scene_metadata['trays'] = tray_geometry
    (out / "scene_materials.json").write_text(json.dumps(scene_metadata, indent=2))
    print(f"[shared-stack] bolt visual materials verified: {color_counts}", flush=True)
    # Settle before defining the episode clock and before connecting the servo.
    for _ in range(1 if frozen else 240):
        world.step(render=False)

    from shared_contact import ToolSensor, ContactRecorder
    sensor = ToolSensor(world, arts, settings, root/settings['servo_config'], arm_indices)
    sensor.save(out/'plant_contact.json')
    if settings['contact']['enabled']:
        for _ in range(round(settings['contact']['settle_sec']/settings['physics_dt_sec'])):
            world.step(render=False)
    initial_bolts = []
    for path in bolt_prims:
        p, q = bolt_views[path].get_world_poses()
        initial_bolts.append(dict(p=np.asarray(p)[0].tolist(),q=np.asarray(q)[0].tolist()))
    if 'size_m' in scene.WORK_SURFACE:
        scene_metadata['support_check'] = scene.work_surface.verify_bolts(
            scene.WORK_SURFACE, scene.TABLE_Z, initial_bolts, scene_metadata['bolt_proxies'])
    scene_metadata['initial_bolts'] = initial_bolts
    (out / 'scene_materials.json').write_text(json.dumps(scene_metadata, indent=2))

    def measure():
        result = {}
        for side, art in arts.items():
            q = np.asarray(art.get_joint_positions())
            dq = np.asarray(art.get_joint_velocities())
            f = q[finger_indices[side]]
            grip = np.clip(100 * (1 - (f[0] - f[1]) / (2 * scene.FINGER_TRAVEL_M)), 0, 100)
            result[side] = dict(q_deg=np.rad2deg(q[arm_indices[side]]).tolist(),
                                dq_deg_s=np.rad2deg(dq[arm_indices[side]]).tolist(),
                                gripper_percent=float(grip))
            if sensor.enabled:
                result[side]['force_sensor'] = sensor.measure(
                    side, clock.now_ns(), 1+(clock.now_ns()-1_000_000_000)//clock.dt_ns)
        return result

    def apply(reply):
        for side, art in arts.items():
            q = np.asarray(art.get_joint_positions()).copy()
            q[arm_indices[side]] = np.deg2rad(reply[side]["q_target_deg"])
            travel = (1 - reply[side]["gripper_percent"] / 100) * scene.FINGER_TRAVEL_M
            q[finger_indices[side]] = [travel, -travel]
            art.apply_action(ArticulationAction(joint_positions=q))

    class Cameras:
        bundle = None
        count = 0
        def poll(self, timeout_ms=0):
            return self.bundle
        def is_fresh(self, bundle):
            return (bundle is not None and bundle.complete and
                    0 <= clock.now_ns() - bundle.bundle_time_ns <= config.camera.max_age_ms * 1e6)
        def close(self):
            pass
        def render(self):
            before = world.current_time_step_index
            rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
            if world.current_time_step_index != before:
                raise RuntimeError("render unexpectedly advanced physics")
            frames = {}
            self.count += 1
            for side, camera in cameras.items():
                rgba = np.asarray(camera.get_rgba())
                if rgba.shape != (480, 640, 4) or not np.any(rgba[..., :3]):
                    raise RuntimeError(f"empty/invalid {side} wrist image: {rgba.shape}")
                rgb = np.ascontiguousarray(rgba[..., :3], dtype=np.uint8)
                name = f"{side}_realsense"
                frames[name + ".color"] = CameraFrame(name, 640, 480, rgb,
                                                       "rgb8", self.count,
                                                       clock.now_ns(), clock.now_ns())
                if self.count == 1:
                    Image.fromarray(rgb).save(out / f"{side}_wrist.png")
            self.bundle = CameraBundle(self.count, clock.now_ns(), False, True,
                                       clock.monotonic(), frames, "simulation_tick", 0.0)

    camera_client = Cameras()
    # Warm annotators without advancing the plant.
    for _ in range(2):
        rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
    camera_client.render()
    bridge = ServoBridge(root / settings["bridge"], root / settings["servo_config"],
                         {**measure(), 'force_sensor_enabled': sensor.enabled},
                         cwd=root, log_path=out / "servo.log")
    video = None
    stream = None
    force_stream = None
    source = None
    chunk_stream = open(out/'chunk_packets.jsonl', 'w')
    contact_recorder = ContactRecorder(out/'contacts.jsonl', clock)
    class RecordedChunkDatagram(PipeDatagram):
        def sendto(self, data, address):
            sent = super().sendto(data, address)
            chunk_stream.write(json.dumps({'t_mono': clock.monotonic(),
                                           'packet': json.loads(data)})+'\n')
            return sent
    started = time.monotonic()
    summary = {"settings": settings, "plant": "isaac_physx", "force_control": sensor.enabled,
               "contact_model": sensor.metadata,
               "scene": scene_metadata,
               # lighting / tonemap / material overrides decide the policy's pixels, so a board
               # scored under one photometry must be tellable from a board scored under another
               "photometry": scene.photometry_metadata(),
               "source_configs": {key: {"path": str(root/settings[key]),
                  "sha256": hashlib.sha256((root/settings[key]).read_bytes()).hexdigest()}
                  for key in ("servo_config", "runner_config")},
               "overrides": bridge.reply["overrides"], "exit_code": None}
    try:
        # Reject wrong frames/assets before allowing the policy to arm.
        from policy_runner.flow_dataset import pose_from_state_payload
        fk_errors = {}
        for side, view in tcp_views.items():
            p, q = view.get_world_poses()
            pose = np.asarray(pose_from_state_payload(bridge.reply["state"], side))
            sim_r = Rotation.from_quat(np.asarray(q)[0][[1, 2, 3, 0]])
            cpp_r = Rotation.from_quat(pose[3:7])
            pos_error = float(np.linalg.norm(np.asarray(p)[0] - pose[:3]))
            ang_error = float(np.rad2deg((sim_r.inv() * cpp_r).magnitude()))
            fk_errors[side] = {"position_m": pos_error, "angle_deg": ang_error}
            if pos_error > settings["fk_position_tolerance_m"] or ang_error > settings["fk_angle_tolerance_deg"]:
                raise RuntimeError(f"{side} PhysX/Pinocchio frame mismatch: {fk_errors[side]}")
        summary["fk_errors"] = fk_errors
        config = load_config(root / settings["runner_config"])
        config = replace(config, mode="simulation", action_source="hold",
                         geometry=replace(config.geometry, path=str(root/config.geometry.path)),
                         recording=replace(config.recording, control_enabled=False, status_endpoint=None),
                         servo_command=replace(config.servo_command, endpoint="udp://127.0.0.1:1", acquire_lease=True),
                         safety=replace(config.safety, allow_real_motion=False, allow_real_gripper_motion=False),
                         camera=replace(config.camera, expected_cameras=["left_realsense_color", "right_realsense_color"]))
        if args.shared_replay:
            from shared_replay import RecordedPolicySource
            source = RecordedPolicySource(args.shared_replay, ChunkOverlayPublisher(
                "udp://127.0.0.1:1", socket_factory=lambda *a: RecordedChunkDatagram(bridge, "chunk")))
            summary['recorded_policy_replay'] = source.metadata
        elif args.shared_hold:
            from policy_runner.action_sources.hold import HoldActionSource
            source = HoldActionSource(send_hold=True, timeout_sec=config.servo_command.timeout_sec)
        else:
            from policy_runner.openpi_remote import OpenpiRemoteActionSource
            checkpoint = scene._resolve_served_checkpoint(args.host, args.port)
            contract = scene._checkpoint_contract(checkpoint.get("dir", "")) if checkpoint.get("resolved") else {}
            if contract.get("action_mode") != settings["action_mode"]:
                raise RuntimeError(f"checkpoint action contract not verified: {contract}")
            summary["checkpoint"] = checkpoint
            summary["checkpoint_contract"] = contract
            source = OpenpiRemoteActionSource(
                f"{args.host}:{args.port}", camera_client=camera_client,
                timeout_sec=config.servo_command.timeout_sec,
                policy_dt_sec=settings["policy_dt_sec"],
                action_horizon=settings["action_horizon"], action_mode=settings["action_mode"],
                proprio_mode=settings["proprio_mode"], velproprio_sample_mode=settings["velproprio_sample_mode"],
                velproprio_source=settings["velproprio_source"], chunk_execute_steps=settings["execute_steps"],
                chunk_overlay_runway_steps=settings["runway_steps"], chunk_crossfade_steps=0,
                tcp_target_pose_conditioning="foh_se3", tcp_target_pose_reanchor_mode="last_emitted_continuous",
                ee_local_r_align="pika_rz180", include_depth=False, rtc_enabled=False,
                clock=clock, chunk_overlay_endpoint="",
            )
            source.configure_camera_runtime(config.camera)
            source.runner_role = source.name = "flow_infer"
            source.chunk_anchor_source = settings["chunk_anchor"]
            source.gripper_proprio_source = settings["gripper_proprio_source"]
            source.gripper_close_bias_left = settings["gripper_close_bias_left"]
            source.gripper_close_bias_right = settings["gripper_close_bias_right"]
            source.enable_async_chunking = source.nonblocking_stream_inference = True
            source._client = TimedInferenceClient(source._client, clock)
            source._chunk_overlay_publisher = ChunkOverlayPublisher(
                "udp://127.0.0.1:1", socket_factory=lambda *a: RecordedChunkDatagram(bridge, "chunk"))
            source.configure_rollout_step_log(str(out / "policy_steps.jsonl"))
        command_client = ServoCommandClient(
            "udp://127.0.0.1:1", timeout_sec=config.servo_command.timeout_sec,
            socket_factory=lambda *a: PipeDatagram(bridge, "command"))
        stream = open(out / "joints.csv", "w")
        writer = csv.writer(stream)
        writer.writerow(["time_ns", "side", "fault", "controller"] +
                        [f"{signal}{j+1}" for signal in ("q_deg_", "target_deg_", "dq_deg_s_") for j in range(6)])
        force_stream = open(out/'force.csv', 'w')
        force_writer = csv.writer(force_stream)
        force_writer.writerow(['time_ns','side','covered','bias_valid','gate','deviation_m','lead_m']+
                              ['fx','fy','fz','tx','ty','tz','gate_force_n','gate_torque_nm',
                               'bounded','folded'])
        if args.video:
            import imageio.v2 as imageio
            video = imageio.get_writer(str(out / "overview.mp4"), fps=1/settings["camera_dt_sec"])
        camera_period_ns = round(settings["camera_dt_sec"] * 1e9)
        next_camera_ns = clock.now_ns() + camera_period_ns
        physics_ticks = 0
        controllers = set()
        active_chunk_ticks = 0
        states = open(out / "states.jsonl", "w")
        def advance(seconds):
            nonlocal next_camera_ns, physics_ticks, active_chunk_ticks
            # Fractional policy/camera periods never round up to 17 substeps.
            steps = max(1, round(max(0, seconds) / settings["physics_dt_sec"]))
            for _ in range(steps):
                apply(bridge.reply)
                before = world.current_time_step_index
                world.step(render=False)
                if world.current_time_step_index != before + 1:
                    raise RuntimeError("PhysX must advance exactly once per servo tick")
                clock.advance()
                physics_ticks += 1
                measured = measure()
                reply = bridge.step(clock, measured)
                for side in arts:
                    controller = reply["state"][side]["shared_control"]["controller"]
                    controllers.add(controller)
                    if reply['state'][side]['shared_control']['active'] and controller == 'delta_preview':
                        active_chunk_ticks += 1
                    writer.writerow([clock.now_ns(), side, reply["state"]["fault_latched"], controller] +
                                    measured[side]["q_deg"] + reply[side]["q_target_deg"] + measured[side]["dq_deg_s"])
                    fc = reply['state'][side]['force_control']
                    ft = reply['state'][side]['force_torque']
                    force_writer.writerow([clock.now_ns(), side, fc['covered'], ft['bias_valid'],
                                           fc['gate_translation'], fc['deviation_norm_m'],
                                           reply['state'][side]['shared_control']['actual_lead_m']]+
                                          fc['wrench_stand_axes_at_tcp']+
                                          [fc['gate_force_n'],fc['gate_torque_nm'],fc['bounded'],fc['folded']])
                if clock.now_ns() >= next_camera_ns:
                    camera_client.render()
                    if video is not None:
                        video.append_data(np.asarray(overview.get_data())[..., :3])
                    states.write(json.dumps(reply["state"]) + "\n")
                    while next_camera_ns <= clock.now_ns():
                        next_camera_ns += camera_period_ns
        try:
            if sensor.enabled:
                # Refuse a bad sign/mass/COM before tare can conceal it as bias.
                for side in arts:
                    ft = bridge.reply['state'][side]['force_torque']
                    residual = np.asarray(ft['comp_sensor_at_sro_nodeadzone'])
                    if (not ft['connected'] or
                        np.linalg.norm(residual[:3]) > settings['contact']['static_force_tolerance_n'] or
                        np.linalg.norm(residual[3:]) > settings['contact']['static_torque_tolerance_nm']):
                        raise RuntimeError(f'{side}: static F/T gravity/sign check failed: {ft}')
                from policy_runner.servo_command_client import CommandIntent
                command_client.send(CommandIntent('TareForceSensor'))
                advance(settings['contact']['tare_sec'])
                for side in arts:
                    ft = bridge.reply['state'][side]['force_torque']
                    if not ft['bias_valid'] or ft['bias_source'] != 'tare':
                        raise RuntimeError(f'{side}: PhysX F/T tare did not complete: {ft}')
                summary['tare'] = {s: bridge.reply['state'][s]['force_torque'] for s in arts}
                print('[shared-stack] physical tool wrench sign/gravity verified; 250-sample tare accepted', flush=True)
            episode_start_ns = clock.now_ns()
            summary['episode_start_time_ns'] = episode_start_ns
            code = run(config, source=BoundedSource(source, clock, args.episode_sec),
                       state_client=BridgeStateClient(bridge, clock), command_client=command_client,
                       sleep_fn=advance, monotonic_fn=clock.monotonic,
                       pacing_monotonic_fn=clock.monotonic)
            # Consume the final Hold/ReleaseLease through the same control tick.
            advance(0.002)
            summary.update(exit_code=code, physics_ticks=physics_ticks,
                           active_chunk_arm_ticks=active_chunk_ticks,
                           sim_elapsed_sec=(clock.now_ns()-1_000_000_000)*1e-9,
                           policy_elapsed_sec=(clock.now_ns()-episode_start_ns)*1e-9,
                           camera_frames=camera_client.count, chunks=bridge.chunk_count,
                           commands=bridge.command_count, controllers=sorted(controllers),
                           fault_latched=bridge.reply["state"]["fault_latched"],
                           fault_reason=bridge.reply["state"]["fault_reason"])
            if args.shared_replay:
                summary['replayed_chunks'] = source.ci
            elif not args.shared_hold:
                summary["inference_timing"] = source._client.calls
                if code == 0 and bridge.chunk_count == 0:
                    raise RuntimeError("policy episode produced no chunk frames")
                if code == 0 and "delta_preview" not in controllers:
                    raise RuntimeError("policy episode never activated the C++ delta_preview controller")
            if code == 0 and not args.shared_hold and active_chunk_ticks == 0:
                raise RuntimeError('episode never activated a valid C++ delta_preview chunk')
            return code
        finally:
            states.close()
    except BaseException as exc:
        summary["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        clock.cancel()
        bridge.close()
        contact_recorder.close()
        chunk_stream.close()
        if stream:
            stream.close()
        if force_stream:
            force_stream.close()
        if video:
            video.close()
        summary["wall_elapsed_sec"] = time.monotonic() - started
        (out / "summary.json").write_text(json.dumps(summary, indent=2))
        print(f"[shared-stack] results: {out}", flush=True)
