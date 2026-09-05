"""Replay recorded policy inputs through the shared controller, never joint targets.

This is an offline contact regression, not another policy implementation or a
closed-loop task score. Original chunk deltas, commands, ordering and timestamps
are retained relative to the recorded episode origin; F/T and safety remain live.
"""
import hashlib
import json
import math
from pathlib import Path


class RecordedPolicySource:
    name = runner_role = 'flow_infer'

    def __init__(self, directory, publisher):
        from policy_runner.action_sources.tcp_pose_target import cartesian_action_requirements
        self.requirements = cartesian_action_requirements()
        directory = Path(directory).resolve()
        # Older chunk-row logs omit the validated v3 alignment/proprio metadata.
        # Do not invent it or silently replay an inactive follower.
        self.chunks = [json.loads(l) for l in (directory/'chunk_packets.jsonl').read_text().splitlines()]
        self.steps = [json.loads(l) for l in (directory/'policy_steps.jsonl').read_text().splitlines()]
        if not self.chunks or not self.steps:
            raise ValueError('recorded policy replay requires nonempty chunk and step logs')
        for rows in (self.chunks, self.steps):
            if any(not math.isfinite(r['t_mono']) for r in rows):
                raise ValueError('recorded policy timestamps must be finite')
            if any(b['t_mono'] < a['t_mono'] for a,b in zip(rows, rows[1:])):
                raise ValueError('recorded policy timestamps must be monotonic')
        if any('chunk_metadata' not in r['packet'] for r in self.chunks):
            raise ValueError('replay requires original v3 chunk metadata')
        if any(r['schema'] != 'robotics_lab.policy_runner.rollout_step.v1' for r in self.steps):
            raise ValueError('unsupported recorded step schema')
        summary = json.loads((directory/'summary.json').read_text())
        self.origin = summary.get('episode_start_time_ns', 1_000_000_000)*1e-9
        self.metadata = {'source': str(directory), 'episode_origin_sec': self.origin,
                         'sha256': {name: hashlib.sha256((directory/name).read_bytes()).hexdigest()
                                    for name in ('chunk_packets.jsonl', 'policy_steps.jsonl')}}
        self.publisher, self.start, self.ci, self.si = publisher, None, 0, -1

    def next_intent(self, snapshot, now_monotonic):
        from policy_runner.action_sources.tcp_pose_target import tcp_pose_target_stand_intent
        from policy_runner.servo_command_client import CommandIntent
        if self.start is None:
            self.start = now_monotonic
        recorded_time = now_monotonic-self.start+self.origin
        while self.ci < len(self.chunks) and self.chunks[self.ci]['t_mono'] <= recorded_time+1e-9:
            row = self.chunks[self.ci]['packet']
            self.publisher.publish(seq=row['seq'], policy_dt_sec=row['policy_dt_sec'],
                                   left=row['left'], right=row['right'],
                                   left_delta=row['left_delta'], right_delta=row['right_delta'],
                                   host_time_ns=round(now_monotonic*1e9),
                                   execute_steps=row['execute_steps'], runway_steps=row['runway_steps'],
                                   chunk_metadata=row['chunk_metadata'])
            self.ci += 1
        while self.si+1 < len(self.steps) and self.steps[self.si+1]['t_mono'] <= recorded_time+1e-9:
            self.si += 1
        if self.si < 0:
            return CommandIntent.hold()
        row = self.steps[self.si]
        if row.get('hold'):
            return CommandIntent.hold()
        return tcp_pose_target_stand_intent(
            left=row['arms']['left']['cmd_pose'], right=row['arms']['right']['cmd_pose'],
            left_gripper=row['arms']['left']['gripper_cmd_pct'],
            right_gripper=row['arms']['right']['gripper_cmd_pct'],
            tcp_target_profile='flow_infer_smooth',
            metadata={'action_source': 'flow_infer', 'source_conditioning_mode': 'foh_se3'})

    def close(self):
        self.publisher.close()
