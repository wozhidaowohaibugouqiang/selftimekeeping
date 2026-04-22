import argparse
import math
from pathlib import Path
import csv
import matplotlib.pyplot as plt

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

def plot_data(out_png: Path, data_signed_list, title_prefix="Flash DATA (Filtered)"):
    if not data_signed_list:
        raise ValueError("No data available to plot after filtering.")

    fig, ax = plt.subplots(1, 1, figsize=(14, 5), constrained_layout=True)

    x = list(range(len(data_signed_list)))
    ax.plot(x, data_signed_list, linewidth=1.2, color="#1f77b4")
    ax.axhline(0, linestyle="--", linewidth=1, color="gray")
    
    stats = calc_stats(data_signed_list)
    annotate_stats(ax, title_prefix, stats)
    
    ax.set_xlabel("Filtered Sample index")
    ax.set_ylabel("Residual")
    ax.grid(alpha=0.25)

    fig.suptitle(f"CSV Extract + Plot ({out_png.name})")
    fig.savefig(out_png, dpi=160)
    plt.close(fig)

def process_csv(csv_path: Path, outdir: Path):
    data_signed_list = []
    
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            data_hex = row.get("data_hex", "")
            # Filter out entries that start with FFFFF (which are usually -1 to -something very small in 32-bit hex)
            # Or you can filter by checking if it starts with '0xFFFFF' or just 'FFFFF'
            # Based on your CSV format: 0xFFFFF66F -> data_hex is "0xFFFFF66F"
            if data_hex.upper().startswith("0XFFFFF") or data_hex.upper().startswith("FFFFF"):
                continue
                
            try:
                val = int(row["data_signed"])
                # Extra safety check, you can also filter by value if you know the typical range
                # if abs(val) < 100000: # example threshold
                data_signed_list.append(val)
            except ValueError:
                continue
                
    if not data_signed_list:
        print(f"Skipping {csv_path.name}: No valid data found after filtering FFFFF.")
        return

    stem = csv_path.stem
    out_png = outdir / f"{stem}_filtered_plot.png"
    
    plot_data(out_png, data_signed_list)
    print(f"Processed {csv_path.name} -> {out_png.name} (Valid items: {len(data_signed_list)})")

def main():
    parser = argparse.ArgumentParser(description="Plot CSV data while ignoring FFFFF... entries")
    parser.add_argument("inputs", nargs="+", help="Input CSV files")
    parser.add_argument("--outdir", "-o", default=".", help="Output directory")
    args = parser.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    for input_file in args.inputs:
        path = Path(input_file)
        if path.exists():
            process_csv(path, outdir)
        else:
            print(f"File not found: {input_file}")

if __name__ == "__main__":
    main()
