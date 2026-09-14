"""Read recorded body poses to verify lift and final tray containment."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation

import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('directory', type=Path)
OUT=parser.parse_args().directory.resolve()
manifest=json.load(open(OUT/'manifest.json'))
results={}
for port,record in manifest['models'].items():
    summary=json.load(open(Path(record['directory'])/'summary.json'))
    poses=[json.loads(l) for l in (Path(record['diagnostics'])/'poses.jsonl').open()]
    origins={name:np.array(v[:3]) for name,v in poses[0]['bolts'].items()}
    all_xyz={name:np.array([r['bolts'][name][:3] for r in poses]) for name in origins}
    all_t=np.array([r['time_ns']*1e-9-summary['episode_start_time_ns']*1e-9 for r in poses])
    bolts={};correct={'gray':[],'black':[]};wrong=[]
    for name,origin in origins.items():
        idx=int(name.split('_')[-1]);color=summary['scene']['bolt_colors'][idx]
        xyz=all_xyz[name];delta=(xyz-origin)*1000
        v=np.array(poses[-1]['bolts'][name]);rotation=Rotation.from_quat(v[[4,5,6,3]])
        axis=rotation.as_matrix()[:,0];bounds=[]
        shapes=(summary['scene']['bolt_proxies'][idx] if 'bolt_proxies' in summary['scene']
                else [(-.006,.006,.0092),(.0125,.0125,.006)])      # (axial centre, half length, radius)
        for x,half,radius in shapes:
            length=2*half
            center=rotation.apply([x,0,0])+v[:3]
            extent=length/2*np.abs(axis)+radius*np.sqrt(np.maximum(0,1-axis**2))
            bounds.append((center-extent,center+extent))
        lo=np.min([b[0] for b in bounds],axis=0);hi=np.max([b[1] for b in bounds],axis=0)
        containers=[]
        for box,data in summary['scene']['trays'].items():
            minimum=np.array(data['min_m']);maximum=np.array(data['max_m']);thickness=data['wall_thickness_m']
            if (np.all(lo[:2]>minimum[:2]+thickness) and np.all(hi[:2]<maximum[:2]-thickness)
                    and lo[2]>=minimum[2]-.001 and hi[2]<maximum[2]):containers.append(box)
        expected='box_gray' if color=='gray' else 'box_green'
        if expected in containers:correct[color].append(name)
        elif containers:wrong.append(name)
        bolts[name]={'color':color,'max_origin_lift_mm':float(delta[:,2].max()),
            'max_lift_policy_s':float(all_t[delta[:,2].argmax()]),
            'final_origin_m':xyz[-1].tolist(),'final_geometry_min_m':lo.tolist(),'final_geometry_max_m':hi.tolist(),
            'final_boxes_containing_whole_bolt':containers}
    results[port]={'policy_seconds':summary['policy_elapsed_sec'],'fault_reason':summary['fault_reason'],
        'final_correct_box_counts':{color:len(names) for color,names in correct.items()},
        'final_correct_box_bolts':correct,'final_wrong_box_bolts':wrong,
        'bolts_lifted_over_30mm':[name for name,r in bolts.items() if r['max_origin_lift_mm']>30],
        'bolts':bolts}
(OUT/'task_metrics.json').write_text(json.dumps(results,indent=2))
print(json.dumps({p:{k:v for k,v in r.items() if k!='bolts'} for p,r in results.items()},indent=2))
