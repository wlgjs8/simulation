"""Verify and package three live policy rollouts on the same foam scene."""
from pathlib import Path
import hashlib
import itertools
import json
import shutil
import subprocess
import sys
import numpy as np
from PIL import Image

import argparse
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('directory', type=Path)
OUT=parser.parse_args().directory.resolve()
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_shared_stack import analyze

manifest=json.loads((OUT/'manifest.json').read_text())
ports=['8001','8002','8003']
summaries={};initial={};durations=[];videos=[]
for port in ports:
    record=manifest['models'][port];directory=ROOT/'outputs/shared_stack'/record['tag']
    summary=json.loads((directory/'summary.json').read_text())
    assert summary['checkpoint']['dir']==record['checkpoint']
    assert summary['active_chunk_arm_ticks']>0 and summary['force_control']
    assert summary['settings']['gripper_close_bias_left']==0 and summary['settings']['gripper_close_bias_right']==0
    assert summary['scene']['support_check']['all_on_surface']
    assert summary['scene']['work_surface']['texture_rgb_scale']==[.70,.75,.95]
    video=OUT/f'{port}.mp4';shutil.copyfile(directory/'overview.mp4',video)
    shutil.copyfile(directory/'summary.json',OUT/f'{port}.summary.json')
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames',
        '-show_entries','stream=width,height,r_frame_rate,nb_read_frames:format=duration',
        '-of','json',str(video)]))
    stream=probe['streams'][0];frames=int(stream['nb_read_frames'])
    assert (stream['width'],stream['height'],stream['r_frame_rate'])==(960,720,'30/1')
    assert frames==summary['camera_frames']-1
    duration=frames/30;durations.append(duration);videos.append(video)
    subprocess.run(['ffmpeg','-v','error','-y','-ss',str(min(12,duration-.2)),
        '-i',str(video),'-frames:v','1',str(OUT/f'{port}.preview.png')],check=True)
    record.update(directory=str(directory),video=str(video),video_probe=probe,server=summary['checkpoint'],
        policy_elapsed_sec=summary['policy_elapsed_sec'],exit_code=summary['exit_code'],
        fault_reason=summary['fault_reason'],chunks=summary['chunks'],metrics=analyze(directory))
    summaries[port]=summary
    initial[port]=json.loads(next((directory/'states.jsonl').open()))

verification={'pairs':{},'current_sources_unchanged':True,'current_assets_unchanged':True}
for path,digest in manifest['source_hashes'].items():
    assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==digest
for path,record in manifest['snapshots'].items():
    assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==record['sha256']
for a,b in itertools.combinations(ports,2):
    sa,sb=summaries[a],summaries[b]
    result={key:sa[key]==sb[key] for key in ['settings','source_configs','contact_model']}
    assert all(result.values())
    scene_a={k:v for k,v in sa['scene'].items() if k not in ['initial_bolts','support_check']}
    scene_b={k:v for k,v in sb['scene'].items() if k not in ['initial_bolts','support_check']}
    result['scene_configuration_identical']=scene_a==scene_b
    assert result['scene_configuration_identical']
    pa=np.array([p['p'] for p in sa['scene']['initial_bolts']])
    pb=np.array([p['p'] for p in sb['scene']['initial_bolts']])
    result['initial_bolt_position_max_difference_mm']=float(np.max(np.abs(pa-pb))*1000)
    qa=np.array([p['q'] for p in sa['scene']['initial_bolts']]);qb=np.array([p['q'] for p in sb['scene']['initial_bolts']])
    dots=np.clip(np.abs(np.sum(qa*qb,axis=1)/(np.linalg.norm(qa,axis=1)*np.linalg.norm(qb,axis=1))),0,1)
    result['initial_bolt_angle_max_difference_deg']=float(np.rad2deg(2*np.arccos(dots)).max())
    assert result['initial_bolt_position_max_difference_mm']<.5
    result['initial_arm_max_difference_deg']=max(float(np.max(np.abs(
        np.array(initial[a][side]['q_actual_deg'])-initial[b][side]['q_actual_deg']))) for side in ['left','right'])
    result['wrist_rgb_mae']={}
    for side in ['left','right']:
        imgs=[np.array(Image.open(Path(manifest['models'][p]['directory'])/f'{side}_wrist.png')).astype(float) for p in [a,b]]
        result['wrist_rgb_mae'][side]=float(np.mean(np.abs(imgs[0]-imgs[1])))
    verification['pairs'][f'{a}_vs_{b}']=result

font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
filters=[]
for i,port in enumerate(ports):
    record=manifest['models'][port]
    chain=f'[{i}:v]setpts=PTS-STARTPTS,scale=640:480,tpad=stop_mode=clone:stop_duration={max(durations)-durations[i]:.6f},pad=640:550:0:42:black'
    chain+=f",drawtext=fontfile={font}:text='{port} - {record['label']}':fontsize=23:fontcolor=white:x=12:y=10"
    chain+=f",drawtext=fontfile={font}:text='Bias L0 / R0 - same foam - seed 100':fontsize=17:fontcolor=white:x=12:y=528"
    if summaries[port]['fault_latched']:
        chain+=f",drawtext=fontfile={font}:text='STOPPED at {record['policy_elapsed_sec']:.3f}s - safety fault':fontsize=18:fontcolor=white:box=1:boxcolor=red@0.8:x=12:y=480:enable='gte(t,{durations[i]-.04:.6f})'"
    filters.append(chain+f'[p{i}]')
filters.append('[p0][p1][p2]hstack=inputs=3:shortest=1[v]')
comparison=OUT/'8001_8002_8003_comparison.mp4'
cmd=['ffmpeg','-v','error','-y']
for video in videos:cmd+=['-i',str(video)]
subprocess.run(cmd+['-filter_complex',';'.join(filters),'-map','[v]','-an','-c:v','libx264',
    '-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(comparison)],check=True)
subprocess.run(['ffmpeg','-v','error','-y','-ss',str(min(12,max(durations)-.2)),
    '-i',str(comparison),'-frames:v','1',str(OUT/'comparison.preview.png')],check=True)
verification['comparison_probe']=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames',
    '-show_entries','stream=width,height,r_frame_rate,nb_read_frames:format=duration',
    '-of','json',str(comparison)]))
manifest['comparison_video']=str(comparison)
manifest['same_scene_verification']=verification
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2))
(OUT/'verification.json').write_text(json.dumps(verification,indent=2))
rows=['| 모델 | 정책 실행 시간 | 제어 결과 | 영상 |','|---|---:|---|---|']
for port in ports:
    r=manifest['models'][port]
    status='fault 없음' if not summaries[port]['fault_latched'] else r['fault_reason']
    rows.append(f"| {port} ({r['label']}) | {r['policy_elapsed_sec']:.3f}초 | {status} | [{port}.mp4]({port}.mp4) |")
readme='''# bias 0 / 0: 보정된 폼 패드에서 8001 / 8002 / 8003 비교

8002와 8003은 이번 요청에서 새로 추론한 폐쇄루프 실행이다.
8001은 직전 검증한 bias 0 실행 foam_grasp_bias0_20260906을 비교에 재사용했다.
세 실행 모두 양손 close bias 0/0이며, 이전 bias 2/6 영상과 구분해야 한다.
같은 aligned seed 100, 회색/검정 각 10개, 500×500×20mm 폼 패드,
보정 RGB 배율 [0.70,0.75,0.95], 같은 reset/카메라/제어 설정을 사용했다.

'''+ '\n'.join(rows)+'''

개별 MP4는 960×720 / 30fps다. 앞 0.6초는 공통 FT tare 구간이다.
[세 모델 나란히 비교](8001_8002_8003_comparison.mp4)는 같은 시뮬레이션 시간으로
정렬했다. 중간에 fault로 끝난 실행이 있다면 마지막 프레임을 유지하고 STOPPED를 표시한다.

8001: boltv2_plain_40k/39999, 8002: boltv2_r5_40k/39999,
8003: boltv2_griponly_40k/39999. 8003은 서버의 학습/서빙 계약에 따라 정규화 후
velocity 12개 차원을 sentinel 1.5로 마스킹하고 gripper 2개 차원을 유지한다.
클라이언트에서는 세 서버에 동일한 14차원 wire 관측 구조를 전달한다.

`verification.json`에는 실행 간 초기 볼트/관절 차이, 초기 이미지 차이,
공통 설정·접촉 모델·장면·소스 일치 검증과 영상 프레임 정보가 있다.
`manifest.json`에는 실제 checkpoint/PID, 설정 및 자산 snapshot/hash, 결과 경로,
추종/힘 지표가 있다. 초기 조건의 일치와 렌더링 픽셀의 bitwise 동일성은 구분한다.

`task_metrics.json`에는 실제 볼트 최대 상승과 최종 박스 내부 여부가 있다.
이 한 장면의 영상으로 일반적인 성공률이나 모델 순위를 확정하지 않는다.
카메라는 앞선 8001과 같은 설정이며, 발견된 Real intrinsics 차이는 아직 보정하지 않았다.
폼 물성은 기존 잠정 Sim 근사값이다. 이번 작업에서 소스 코드, 환경, 제어 설정,
Real 하드웨어와 정책 서버는 변경하지 않았다. 산출물 및 실행 로그만 추가했다.
'''
(OUT/'README.md').write_text(readme)
print(json.dumps({'models':{p:{k:manifest['models'][p][k] for k in ['video','policy_elapsed_sec','exit_code','fault_reason']} for p in ports},'verification':verification},indent=2))
