"""Reference luminance histogram of the deployment wrist stream, per arm.

A six-number region summary let every fit hide a large railed block behind a percentile; the
whole-frame CDF cannot -- a blown-out table shows up as mass at the top bin no matter where it is.
"""
import cv2, numpy as np, glob, os, json
ROOT="/home/plaif/workspace/robotics_lab/logs/flow_obs_am"
BINS=np.arange(0,257,4)              # 64 bins
def lum(a):
    a=a.astype(np.float32); return 0.2126*a[...,2]+0.7152*a[...,1]+0.0722*a[...,0]
out={}
for arm in ("left","right"):
    H=np.zeros(len(BINS)-1); n=0
    for r in sorted(glob.glob(ROOT+"/run_*")):
        fs=sorted(glob.glob(f"{r}/*_{arm}.jpg"))
        if len(fs)<30: continue
        for f in fs[::max(1,len(fs)//10)][:10]:
            im=cv2.imread(f)
            if im is None: continue
            H+=np.histogram(lum(im),bins=BINS)[0]; n+=1
    H/=H.sum(); out[arm]=H.tolist()
    c=np.cumsum(H)
    print(f"{arm}: n={n} frames  p10={BINS[np.searchsorted(c,.10)]:>3} "
          f"p50={BINS[np.searchsorted(c,.50)]:>3} p90={BINS[np.searchsorted(c,.90)]:>3} "
          f"p99={BINS[np.searchsorted(c,.99)]:>3}  top-bin(>=252)={H[-1]*100:.2f}%")
json.dump({"bins":BINS.tolist(),"hist":out}, open("/tmp/real_hist.json","w"))
print("wrote /tmp/real_hist.json")
