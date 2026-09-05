"""Offline regression of recorded chunk timing and v3 metadata preservation."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'robotics_lab/policy_runner'))
from shared_replay import RecordedPolicySource


class Publisher:
    def __init__(self):
        self.calls = []
    def publish(self, **kwargs):
        self.calls.append(kwargs)
    def close(self):
        pass


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = Path(self.tmp.name)
        pose = [0.5,0.2,0.0,0,0,0,1]
        self.metadata = dict(observation_step_seq=4, activation_step_seq=6,
                             source_start_index=2, original_horizon=24, selected_horizon=22,
                             proprio={'valid':True})
        packet = dict(seq=1,policy_dt_sec=0.0334,left=[pose+[50]],right=[pose+[60]],
                      left_delta=[[0]*6+[50]],right_delta=[[0]*6+[60]],
                      execute_steps=4,runway_steps=4,chunk_metadata=self.metadata)
        self.chunks = [dict(t_mono=1.8,packet=packet)]
        self.steps = [dict(schema='robotics_lab.policy_runner.rollout_step.v1',t_mono=1.8,
                           arms={side:dict(cmd_pose=pose,gripper_cmd_pct=g)
                                 for side,g in [('left',50),('right',60)]})]
        self.write()

    def tearDown(self):
        self.tmp.cleanup()

    def write(self):
        (self.directory/'chunk_packets.jsonl').write_text('\n'.join(map(json.dumps,self.chunks)))
        (self.directory/'policy_steps.jsonl').write_text('\n'.join(map(json.dumps,self.steps)))
        (self.directory/'summary.json').write_text(json.dumps({'episode_start_time_ns':1_600_000_000}))

    def test_timing_metadata_and_delta_payload_are_preserved(self):
        publisher = Publisher()
        source = RecordedPolicySource(self.directory,publisher)
        self.assertEqual(source.next_intent(None,10.0).mode,'Hold')
        source.next_intent(None,10.198)
        self.assertEqual(publisher.calls,[])
        intent = source.next_intent(None,10.2)
        self.assertEqual(intent.mode,'TcpPoseTarget')
        self.assertEqual(intent.tcp_target_profile,'flow_infer_smooth')
        self.assertEqual(publisher.calls[0]['chunk_metadata'],self.metadata)
        self.assertEqual(publisher.calls[0]['left_delta'],self.chunks[0]['packet']['left_delta'])
        self.assertEqual(publisher.calls[0]['host_time_ns'],10_200_000_000)
        source.next_intent(None,10.202)
        self.assertEqual(len(publisher.calls),1)

    def test_missing_alignment_metadata_is_not_invented(self):
        del self.chunks[0]['packet']['chunk_metadata']
        self.write()
        with self.assertRaisesRegex(ValueError,'original v3'):
            RecordedPolicySource(self.directory,Publisher())

    def test_invalid_recorded_proprio_is_never_promoted_to_valid(self):
        self.chunks[0]['packet']['chunk_metadata']['proprio']['valid'] = False
        self.write()
        publisher = Publisher()
        source = RecordedPolicySource(self.directory,publisher)
        source.next_intent(None,10.0)
        source.next_intent(None,10.2)
        self.assertFalse(publisher.calls[0]['chunk_metadata']['proprio']['valid'])

    def test_nonmonotonic_input_is_rejected(self):
        self.chunks.append(dict(t_mono=1.7,packet=self.chunks[0]['packet']))
        self.write()
        with self.assertRaisesRegex(ValueError,'monotonic'):
            RecordedPolicySource(self.directory,Publisher())


if __name__ == '__main__':
    unittest.main()
