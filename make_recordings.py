"""Shrink the full-size demo GIFs into GitHub-friendly copies under
docs/recordings/ (the originals are 18-106 MB and gitignored, so the README
could not show them). Frames are subsampled and downscaled, and each copy
is kept under MAX_MB; the motion itself is unchanged.

    .venv\\Scripts\\python.exe make_recordings.py

Regenerate the sources first (render_offscreen.py, render_offscreen.py
--preempt, run_autonomous.py --render, run_gauntlet.py).
"""
from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageSequence

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "docs" / "recordings"
MAX_MB = 9.0

SOURCES = {
    "navigation.gif": ROOT / "data/processed/voice_demo/navigation.gif",
    "preemption.gif": ROOT / "data/processed/voice_demo/robustness_preemption.gif",
    "autonomous_patrol.gif": ROOT / "data/processed/autonomy/autonomous_patrol.gif",
    "gauntlet.gif": ROOT / "results/maneuvers/gauntlet_full.gif",
}


def shrink(src: Path, dst: Path, width: int, step: int) -> float:
    with Image.open(src) as im:
        dur = im.info.get("duration", 40)
        frames = []
        for i, fr in enumerate(ImageSequence.Iterator(im)):
            if i % step:
                continue
            fr = fr.convert("RGB")
            h = round(fr.height * width / fr.width)
            frames.append(fr.resize((width, h), Image.LANCZOS)
                          .quantize(colors=96, method=Image.Quantize.MEDIANCUT))
    frames[0].save(dst, save_all=True, append_images=frames[1:], duration=dur * step,
                   loop=0, optimize=True, disposal=1)
    return dst.stat().st_size / 1e6


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    for name, src in SOURCES.items():
        if not src.exists():
            print(f"  missing {src} - skipped")
            continue
        for width, step in ((640, 2), (560, 3), (480, 4), (420, 6), (360, 8)):
            mb = shrink(src, OUT / name, width, step)
            if mb <= MAX_MB:
                break
        print(f"  {name}: {mb:.1f} MB ({width}px, every {step} frames) from {src.stat().st_size / 1e6:.0f} MB")


if __name__ == "__main__":
    main()
