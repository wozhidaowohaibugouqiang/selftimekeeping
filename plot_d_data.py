import argparse
import math
import re
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt

# 匹配 D: 后的带符号的十六进制数字 (如 D:-187C, D:+0008, D:-0031)
D_VALUE_RE = re.compile(r"D:([+-][0-9A-Fa-f]+)")

def parse_file(path: Path):
    d_list = []

    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = D_VALUE_RE.search(line)
            if m:
                s = m.group(1)
                # s contains the sign and the hex value, e.g. "+0008" or "-187C"
                sign = s[0]
                hex_part = s[1:]
                
                # 过滤异常数据：如果十六进制部分以 FFFF 开头，则跳过不画
                if hex_part.upper().startswith("FFFF"):
                    continue

                val = int(hex_part, 16)
                if sign == "-":
                    val = -val
                d_list.append(val)

    return d_list

def calc_stats(values):
    if not values:
        return None
    n = len(values)
    mean = sum(values) / n
    vmin = min(values)
    vmax = max(values)
    p2p = vmax - vmin
    if n > 1:
        var = sum((x - mean) ** 2 for x in values) / (n - 1)
        std = math.sqrt(var)
    else:
        std = 0.0
    return {
        "n": n,
        "mean": mean,
        "std": std,
        "min": vmin,
        "max": vmax,
        "p2p": p2p,
    }

def annotate_stats(ax, title_prefix, stats):
    title = (
        f"{title_prefix} | mean={stats['mean']:.3f}, std={stats['std']:.3f}, "
        f"p2p={stats['p2p']}, min={stats['min']}, max={stats['max']}, n={stats['n']}"
    )
    ax.set_title(title)

def plot_data(out_png: Path, d_list):
    if not d_list:
        raise ValueError("No D values parsed. Please check the log format.")

    fig, ax = plt.subplots(1, 1, figsize=(14, 5), constrained_layout=True)

    x = list(range(len(d_list)))
    ax.plot(x, d_list, linewidth=1.2, color="#1f77b4")
    ax.axhline(0, linestyle="--", linewidth=1, color="gray")
    
    stats = calc_stats(d_list)
    annotate_stats(ax, "Residual D Value (int32)", stats)
    
    ax.set_xlabel("Sample index")
    ax.set_ylabel("Residual D (clocks)")
    ax.grid(alpha=0.25)

    fig.suptitle("Serial Port D-Value Extract + Plot")
    fig.savefig(out_png, dpi=160)
    plt.close(fig)

def main():
    parser = argparse.ArgumentParser(description="Extract D value from txt and plot")
    parser.add_argument("--input", "-i", help="Input log txt path")
    parser.add_argument("--outdir", "-o", default=".", help="Output directory")
    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    input_path = Path(args.input) if args.input else None
    if not input_path:
        raise SystemExit("Please provide --input")

    d_list = parse_file(input_path)

    stem = input_path.stem
    out_png = outdir / f"{stem}_D_plot.png"

    if d_list:
        plot_data(out_png, d_list)

    print(f"Input file: {input_path}")
    print(f"D Value count: {len(d_list)}")

    if d_list:
        s = calc_stats(d_list)
        print(
            f"D stats: mean={s['mean']:.3f}, std={s['std']:.3f}, "
            f"p2p={s['p2p']}, min={s['min']}, max={s['max']}"
        )
    print(f"Plot: {out_png}")

if __name__ == "__main__":
    main()
