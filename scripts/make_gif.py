"""Turn a Webots recording (scripts/benchmark.py --record) into the README GIF.

    python scripts/make_gif.py flight.mp4 docs/flight.gif --crop 1280:440:0:170 --caption "..."

Uses the ffmpeg that ships with Webots on Windows (or any ffmpeg on the PATH). Two passes:
a palette computed on the clip, then the GIF with that palette, so colours stay clean at a
small file size.
"""
import argparse
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

WEBOTS_FFMPEG = Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/Webots/msys64/mingw64/bin/ffmpeg.exe"


def ffmpeg() -> str:
    found = shutil.which("ffmpeg")
    if found:
        return found
    if WEBOTS_FFMPEG.exists():
        return str(WEBOTS_FFMPEG)
    raise SystemExit("ffmpeg not found (neither on the PATH nor in the Webots installation)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("video")
    ap.add_argument("gif")
    ap.add_argument("--start", type=float, default=0.0, help="[s] into the video")
    ap.add_argument("--duration", type=float, default=12.0, help="[s]")
    ap.add_argument("--width", type=int, default=720)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed factor")
    ap.add_argument("--crop", default="", help="ffmpeg crop w:h:x:y in the source pixels, e.g. 1280:440:0:170")
    ap.add_argument("--caption", default="")
    ap.add_argument("--font", default="C:/Windows/Fonts/segoeui.ttf")
    ap.add_argument("--fontsize", type=int, default=19)
    args = ap.parse_args()

    base = (f"crop={args.crop}," if args.crop else "") + \
        f"setpts=PTS/{args.speed},fps={args.fps},scale={args.width}:-1:flags=lanczos"
    if args.caption:
        font = args.font.replace(":", r"\:")
        text = args.caption.replace(":", r"\:").replace("'", "")
        base += (f",drawtext=fontfile='{font}':text='{text}':x=14:y=h-th-14:fontsize={args.fontsize}:fontcolor=white"
                 ":box=1:boxcolor=black@0.55:boxborderw=8")
    trim = ["-ss", str(args.start), "-t", str(args.duration * args.speed), "-i", args.video]
    with tempfile.TemporaryDirectory() as tmp:
        palette = str(Path(tmp) / "palette.png")
        subprocess.run([ffmpeg(), "-y", "-v", "error", *trim, "-vf", f"{base},palettegen=max_colors=128:stats_mode=diff",
                        palette], check=True)
        subprocess.run([ffmpeg(), "-y", "-v", "error", *trim, "-i", palette, "-lavfi",
                        f"{base}[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
                        "-loop", "0", args.gif], check=True)
    print(f"{args.gif}: {Path(args.gif).stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
