import argparse

import math

import re

from pathlib import Path

from typing import Optional



import matplotlib.pyplot as plt



ADDR_DATA_RE = re.compile(r"ADDR=([0-9A-Fa-f]{1,8}),DATA=([0-9A-Fa-f]{1,8})")

DIFF_RE = re.compile(r"DIFF:([+-][0-9A-Fa-f]{1,8})")





def hex_to_int32(hex_str: str) -> int:

    v = int(hex_str, 16)

    if v & 0x80000000:

        v -= 0x100000000

    return v





def parse_file(path: Path):

    addr_list = []

    data_hex_list = []

    data_signed_list = []

    diff_list = []



    with path.open("r", encoding="utf-8", errors="ignore") as f:

        for line in f:

            m = ADDR_DATA_RE.search(line)

            if m:

                addr_hex, data_hex = m.groups()

                addr = int(addr_hex, 16)

                data_signed = hex_to_int32(data_hex)
                
                # 过滤掉 FFFFF 开头的数据（极大负数/异常数据）
                if data_hex.upper().startswith("FFFFF"):
                    continue

                addr_list.append(addr)
                data_hex_list.append(data_hex.upper())
                data_signed_list.append(data_signed)



            d = DIFF_RE.search(line)

            if d:

                s = d.group(1)

                if s.startswith("+"):

                    diff_list.append(int(s[1:], 16))

                else:

                    diff_list.append(-int(s[1:], 16))



    return addr_list, data_hex_list, data_signed_list, diff_list





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





def save_csv(out_csv: Path, addr_list, data_hex_list, data_signed_list):

    with out_csv.open("w", encoding="utf-8") as f:

        f.write("index,addr_hex,addr_dec,data_hex,data_signed\n")

        for i, (a, h, s) in enumerate(zip(addr_list, data_hex_list, data_signed_list)):

            f.write(f"{i},0x{a:06X},{a},0x{h},{s}\n")





def annotate_stats(ax, title_prefix, stats):

    title = (

        f"{title_prefix} | mean={stats['mean']:.3f}, std={stats['std']:.3f}, "

        f"p2p={stats['p2p']}, min={stats['min']}, max={stats['max']}, n={stats['n']}"

    )

    ax.set_title(title)





def plot_data(out_png: Path, data_signed_list, diff_list):

    has_flash = len(data_signed_list) > 0

    has_diff = len(diff_list) > 0



    if not has_flash and not has_diff:

        raise ValueError("No ADDR/DATA or DIFF parsed. Please check the log format.")



    rows = 2 if has_flash and has_diff else 1

    fig, axes = plt.subplots(rows, 1, figsize=(14, 5 * rows), constrained_layout=True)

    if rows == 1:

        axes = [axes]



    ax_i = 0

    if has_flash:

        ax = axes[ax_i]

        x = list(range(len(data_signed_list)))

        ax.plot(x, data_signed_list, linewidth=1.2, color="#1f77b4")

        ax.axhline(0, linestyle="--", linewidth=1, color="gray")

        flash_stats = calc_stats(data_signed_list)

        annotate_stats(ax, "Flash DATA (signed int32)", flash_stats)

        ax.set_xlabel("Sample index")

        ax.set_ylabel("Residual")

        ax.grid(alpha=0.25)

        ax_i += 1



    if has_diff:

        ax = axes[ax_i]

        x = list(range(len(diff_list)))

        ax.plot(x, diff_list, linewidth=1.2, color="#d62728")

        ax.axhline(0, linestyle="--", linewidth=1, color="gray")

        diff_stats = calc_stats(diff_list)

        annotate_stats(ax, "DIFF series", diff_stats)

        ax.set_xlabel("Sample index")

        ax.set_ylabel("DIFF")

        ax.grid(alpha=0.25)



    fig.suptitle("Log Extract + Plot")

    fig.savefig(out_png, dpi=160)

    plt.close(fig)





def pick_txt_file(initial_dir: Path) -> Optional[Path]:

    try:

        import tkinter as tk

        from tkinter import filedialog

    except Exception:

        return None



    root = tk.Tk()

    root.withdraw()

    file_path = filedialog.askopenfilename(

        title="Select one TXT log file",

        initialdir=str(initial_dir),

        filetypes=[("Text files", "*.txt"), ("All files", "*.*")],

    )

    root.destroy()



    if not file_path:

        return None

    return Path(file_path)





def main():

    parser = argparse.ArgumentParser(description="Extract ADDR/DATA + DIFF from one txt and plot")

    parser.add_argument("--input", "-i", help="Input log txt path")

    parser.add_argument("--pick", action="store_true", help="Pick one txt via file dialog")

    parser.add_argument("--outdir", "-o", default=".", help="Output directory")

    parser.add_argument("--show", action="store_true", help="Show result image")

    args = parser.parse_args()



    outdir = Path(args.outdir)

    outdir.mkdir(parents=True, exist_ok=True)



    input_path = None

    if args.pick:

        input_path = pick_txt_file(Path.cwd())

        if input_path is None:

            raise SystemExit("No file selected.")

    elif args.input:

        input_path = Path(args.input)

    else:

        raise SystemExit("Please provide --input or use --pick.")



    addr_list, data_hex_list, data_signed_list, diff_list = parse_file(input_path)



    stem = input_path.stem

    out_csv = outdir / f"{stem}_flash_data.csv"

    out_png = outdir / f"{stem}_plot.png"



    if data_signed_list:

        save_csv(out_csv, addr_list, data_hex_list, data_signed_list)



    plot_data(out_png, data_signed_list, diff_list)



    print(f"Input file: {input_path}")

    print(f"ADDR/DATA count: {len(data_signed_list)}")

    print(f"DIFF count: {len(diff_list)}")



    if data_signed_list:

        s = calc_stats(data_signed_list)

        print(

            f"Flash stats: mean={s['mean']:.3f}, std={s['std']:.3f}, "

            f"p2p={s['p2p']}, min={s['min']}, max={s['max']}"

        )

        print(f"CSV: {out_csv}")



    if diff_list:

        s = calc_stats(diff_list)

        print(

            f"DIFF stats : mean={s['mean']:.3f}, std={s['std']:.3f}, "

            f"p2p={s['p2p']}, min={s['min']}, max={s['max']}"

        )



    print(f"Plot: {out_png}")



    if args.show:

        img = plt.imread(out_png)

        plt.figure(figsize=(12, 7))

        plt.imshow(img)

        plt.axis("off")

        plt.title(out_png.name)

        plt.show()





if __name__ == "__main__":

    main()

