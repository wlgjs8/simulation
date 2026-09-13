"""Montage and photometry numbers for a photometry ladder (scripts/photometry_ladder.py) against
the deployment wrist stream.

Montage: one image per picks file. Each row is one wrist at one moment; the first column is a
deployment frame of the same arm in the same phase (robotics_lab flow_obs_am, the policy's input
JPEG), and the remaining columns are the ladder variants of ONE sim snapshot, which share the
geometry pixel for pixel. Every panel is labelled with its own luminance p10/p50/p90 and the
fraction of pixels at >=252. A row may add "real_crop"/"sim_crop" boxes (x0,y0,x1,y1 in 640x480
pixels); those rows are also emitted enlarged into a second "_crops" image.

Picks (JSON):
  {"real_root": ".../robotics_lab/logs/flow_obs_am",
   "ladder_root": ".../ladder",
   "rows": [{"arm": "left", "phase": "init", "real": "run_20260910_165659/0", "sim": "s100/s000"}, ...]}
"real" is <run dir>/<index into that run's sorted *_<arm>.jpg list>.

Numbers (--stats): pooled luminance histogram of every snapshot of every ladder dir given, per
variant and arm, against the deployment reference histogram (the one the 2026-09-12 fit used):
1-Wasserstein distance in luminance units, quantiles, saturated fraction.
"""
import argparse
import glob
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
FONT = "/usr/share/fonts/truetype/nanum/NanumSquareB.ttf"
BINS = np.arange(0, 257, 4)
ORDER = ["A_stock", "B_tonemap", "C_light", "D_fit"]
TITLES = {"real": "실기 (flow_obs_am)", "A_stock": "A 기존 리그", "B_tonemap": "B +톤매퍼 op0",
          "C_light": "C +조명 (dome·창문 sun)", "D_fit": "D +재질 = t013"}


def lum(rgb):
    a = np.asarray(rgb, dtype=np.float32)
    return 0.2126 * a[..., 0] + 0.7152 * a[..., 1] + 0.0722 * a[..., 2]


def quant(y):
    return (np.percentile(y, 10), np.percentile(y, 50), np.percentile(y, 90), (y >= 252).mean() * 100)


def font(size):
    try:
        return ImageFont.truetype(FONT, size)
    except OSError:
        return ImageFont.load_default()


def real_frame(root, spec, arm):
    run, idx = spec.rsplit("/", 1)
    files = sorted(glob.glob(str(Path(root) / run / f"*_{arm}.jpg")))
    return Image.open(files[int(idx)]).convert("RGB"), Path(files[int(idx)]).name


def panel(img, title, sub, w, h, crop=None, sat_overlay=False):
    if crop:
        img = img.crop(tuple(crop))
    y = lum(img)
    p10, p50, p90, sat = quant(y)
    if sat_overlay:
        # where the frame rails: >=252 painted magenta over a dimmed copy, so the eye can tell
        # which OBJECT carries the saturated mass, not just how much of it there is
        a = (np.asarray(img, dtype=np.float32) * 0.45).astype(np.uint8)
        a[y >= 252] = (255, 0, 255)
        img = Image.fromarray(a)
    out = img.resize((w, h), Image.LANCZOS)
    d = ImageDraw.Draw(out)
    txt = f"{title}\n{sub}p10 {p10:.0f}  p50 {p50:.0f}  p90 {p90:.0f}  포화 {sat:.1f}%"
    d.multiline_text((7, 5), txt, font=font(15), fill=(0, 0, 0), spacing=3, stroke_width=3,
                     stroke_fill=(0, 0, 0))
    d.multiline_text((7, 5), txt, font=font(15), fill=(255, 255, 90), spacing=3)
    return out


def montage(picks_path, out_path, w=480, h=360, crops_only=False, sat_overlay=False):
    picks = json.loads(Path(picks_path).read_text())
    rows = [r for r in picks["rows"] if (not crops_only or r.get("real_crop"))]
    if not rows:
        return None
    cw, ch = (w, h) if not crops_only else (480, 336)   # crop boxes are 320x224
    head = 34
    sheet = Image.new("RGB", (5 * cw, head + len(rows) * (ch + 22)), (24, 24, 24))
    d = ImageDraw.Draw(sheet)
    for c, key in enumerate(["real", *ORDER]):
        d.text((c * cw + 8, 7), TITLES[key], font=font(20), fill=(235, 235, 235))
    for r, row in enumerate(rows):
        y0 = head + r * (ch + 22)
        arm = row["arm"]
        real, name = real_frame(picks["real_root"], row["real"], arm)
        snap = Path(picks["ladder_root"]) / row["sim"]
        meta = next((json.loads(l) for l in (snap.parent / "ladder.jsonl").read_text().splitlines()
                     if json.loads(l)["snapshot"] == int(snap.name[1:])), None)
        a = meta["arms"][arm] if meta else {}
        caption = (f"{arm} · {row['phase']}  |  실기 {row['real']}  |  sim {row['sim']} "
                   f"t={meta['sim_time_s']:.1f}s grip {a.get('gripper_pct', 0):.0f}% "
                   f"최근접 {a.get('nearest_color', '?')} {a.get('nearest_dist_m', 0) * 1000:.0f}mm"
                   if meta else f"{arm} · {row['phase']}")
        d.text((8, y0 + 2), caption, font=font(15), fill=(200, 200, 200))
        crop_r = row.get("real_crop") if crops_only else None
        crop_s = row.get("sim_crop") if crops_only else None
        sheet.paste(panel(real, "실기", "", cw, ch, crop_r, sat_overlay), (0, y0 + 22))
        for c, lab in enumerate(ORDER, start=1):
            img = Image.open(snap / f"{lab}_{arm}.png").convert("RGB")
            sheet.paste(panel(img, lab, "", cw, ch, crop_s, sat_overlay), (c * cw, y0 + 22))
    sheet.save(out_path, quality=92)
    return out_path


def stats(ladder_dirs, ref_path):
    ref = json.loads(Path(ref_path).read_text())
    bins = np.asarray(ref["bins"], dtype=float)
    out = {}
    for arm in ("left", "right"):
        ref_h = np.asarray(ref["hist"][arm])
        ref_c = np.cumsum(ref_h)
        rq = {q: float(bins[np.searchsorted(ref_c, q)]) for q in (0.10, 0.50, 0.90)}
        out[arm] = {"reference": {"p10": rq[0.10], "p50": rq[0.50], "p90": rq[0.90],
                                  "sat_pct": round(float(ref_h[-1] * 100), 2)}}
        for lab in ORDER:
            H = np.zeros(len(bins) - 1)
            n = 0
            for d in ladder_dirs:
                for f in sorted(glob.glob(str(Path(d) / "s*" / f"{lab}_{arm}.png"))):
                    H += np.histogram(lum(Image.open(f).convert("RGB")), bins=bins)[0]
                    n += 1
            if not n:
                continue
            h = H / H.sum()
            c = np.cumsum(h)
            out[arm][lab] = {"frames": n,
                             "w1": round(float(np.abs(c - ref_c).sum() * (bins[1] - bins[0])), 2),
                             "p10": float(bins[np.searchsorted(c, .10)]),
                             "p50": float(bins[np.searchsorted(c, .50)]),
                             "p90": float(bins[np.searchsorted(c, .90)]),
                             "sat_pct": round(float(h[-1] * 100), 2)}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--picks", type=Path)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--stats", nargs="*", type=Path, help="ladder dirs to pool")
    ap.add_argument("--reference", type=Path,
                    default=ROOT / "docs/results/20260912/photometry_fit/real_hist.json")
    args = ap.parse_args()
    if args.picks:
        print(montage(args.picks, args.out))
        crops = montage(args.picks, args.out.with_name(args.out.stem + "_crops" + args.out.suffix),
                        crops_only=True)
        if crops:
            print(crops)
        print(montage(args.picks, args.out.with_name(args.out.stem + "_saturation" + args.out.suffix),
                      sat_overlay=True))
    if args.stats:
        print(json.dumps(stats(args.stats, args.reference), indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
