import glob, json, numpy as np, imageio.v2 as imageio, cv2, tempfile, os
BINS = np.arange(0, 257, 4)
def lum_rgb(a): a = a.astype(np.float32); return 0.2126*a[...,0]+0.7152*a[...,1]+0.0722*a[...,2]
def stats(H):
    h = H / H.sum(); c = np.cumsum(h)
    return h, c, {q: int(BINS[np.searchsorted(c, q)]) for q in (.1, .5, .9)}, round(float(h[-1]*100), 2)
for lab in ("A_stock", "D_fit"):
    for arm in ("left", "right"):
        fs = sorted(glob.glob(f"/home/plaif/ladder/s101/s*/{lab}_{arm}.png"))
        frames = [imageio.imread(f)[..., :3] for f in fs]
        Hp = sum(np.histogram(lum_rgb(f), bins=BINS)[0] for f in frames).astype(float)
        tmp = tempfile.mktemp(suffix=".mp4")
        w = imageio.get_writer(tmp, fps=30)          # the writer record_shared_stack.py uses
        for f in frames:
            for _ in range(3): w.append_data(f)       # a few repeats, like consecutive ticks
        w.close()
        cap = cv2.VideoCapture(tmp); Hv = np.zeros(len(BINS)-1)
        while True:
            ok, fr = cap.read()
            if not ok: break
            y = 0.2126*fr[...,2] + 0.7152*fr[...,1] + 0.0722*fr[...,0]
            Hv += np.histogram(y.astype(np.float32), bins=BINS)[0]
        os.remove(tmp)
        hp, cp, qp, sp = stats(Hp); hv, cv, qv, sv = stats(Hv)
        w1 = float(np.abs(cp - cv).sum() * 4)
        print(f"{lab:8s} {arm:5s} n={len(frames)}  PNG q{qp} sat {sp}%  |  mp4 q{qv} sat {sv}%  |  W1(png,mp4)={w1:.2f}")
