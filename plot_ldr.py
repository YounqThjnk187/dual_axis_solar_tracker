#!/usr/bin/env python3
"""
Plot data collected by ldr_logger.py.

Expected CSV columns:
phase, pc_timestamp, phase_elapsed_s, esp_millis,
TL_raw, TR_raw, BL_raw, BR_raw

Install:
    py -m pip install pandas matplotlib

Run:
    py plot_ldr.py --csv ldr_data_YYYYMMDD_HHMMSS.csv
    py plot_ldr.py --csv ldr_data.csv --save

The script creates:
    1) ldr_timeseries.png  - all four LDR readings over time
    2) ldr_phase_boxplot.png - reading distributions by test phase
    3) ldr_phase_means.png - mean ADC reading for each sensor and phase
"""

import argparse
from pathlib import Path
import sys

import pandas as pd
import matplotlib.pyplot as plt


SENSORS = ["TL_raw", "TR_raw", "BL_raw", "BR_raw"]
PHASE_ORDER = ["DARK", "TOP_LEFT", "TOP_RIGHT", "BOTTOM_LEFT", "BOTTOM_RIGHT"]
SENSOR_LABELS = {
    "TL_raw": "Top Left",
    "TR_raw": "Top Right",
    "BL_raw": "Bottom Left",
    "BR_raw": "Bottom Right",
}


def load_data(csv_path: Path) -> pd.DataFrame:
    if not csv_path.exists():
        raise FileNotFoundError(f"Cannot find CSV file: {csv_path}")

    df = pd.read_csv(csv_path)

    required = ["phase"] + SENSORS
    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(
            "CSV is missing required column(s): "
            + ", ".join(missing)
            + "\nColumns found: "
            + ", ".join(df.columns.astype(str))
        )

    for col in SENSORS:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df["phase"] = df["phase"].astype(str).str.strip().str.upper()
    df = df.dropna(subset=SENSORS, how="all").copy()

    if df.empty:
        raise ValueError("No valid LDR samples were found in the CSV.")

    # Keep the intended test sequence, then append any unexpected phase labels.
    present = list(df["phase"].dropna().unique())
    order = [p for p in PHASE_ORDER if p in present]
    order += [p for p in present if p not in order]
    df["phase"] = pd.Categorical(df["phase"], categories=order, ordered=True)

    # Prefer ESP32 millis for a continuous x-axis; it naturally spans phases.
    if "esp_millis" in df.columns:
        df["time_s"] = pd.to_numeric(df["esp_millis"], errors="coerce") / 1000.0
    elif "pc_timestamp" in df.columns:
        timestamps = pd.to_datetime(df["pc_timestamp"], errors="coerce")
        if timestamps.notna().any():
            df["time_s"] = (
                timestamps - timestamps.dropna().iloc[0]
            ).dt.total_seconds()
        else:
            df["time_s"] = range(len(df))
    else:
        df["time_s"] = range(len(df))

    # If ESP32 millis wrapped/reset or is unusable, use sample index.
    if df["time_s"].isna().all():
        df["time_s"] = range(len(df))
    else:
        df["time_s"] = df["time_s"].fillna(method="ffill").fillna(0)

    return df


def plot_timeseries(df: pd.DataFrame, output_dir: Path, show: bool):
    fig, ax = plt.subplots(figsize=(13, 6))

    for sensor in SENSORS:
        ax.plot(
            df["time_s"],
            df[sensor],
            label=SENSOR_LABELS[sensor],
            linewidth=1.2,
            alpha=0.9,
        )

    # Mark transitions between phases with vertical dashed lines.
    phase_values = df["phase"].astype(str).to_numpy()
    times = df["time_s"].to_numpy()
    for i in range(1, len(df)):
        if phase_values[i] != phase_values[i - 1]:
            ax.axvline(times[i], linestyle="--", linewidth=0.8, alpha=0.45)

    ax.set_title("Four LDR Sensors - Raw ADC Readings Over Time")
    ax.set_xlabel("Time (s, ESP32 millis)")
    ax.set_ylabel("ADC Reading")
    ax.grid(True, alpha=0.25)
    ax.legend(ncol=4)
    fig.tight_layout()
    fig.savefig(output_dir / "ldr_timeseries.png", dpi=160)

    if show:
        plt.show()
    plt.close(fig)


def plot_boxplot(df: pd.DataFrame, output_dir: Path, show: bool):
    phases = [str(p) for p in df["phase"].cat.categories]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), sharex=True)
    axes = axes.flatten()

    for ax, sensor in zip(axes, SENSORS):
        data = [
            df.loc[df["phase"].astype(str) == phase, sensor].dropna().values
            for phase in phases
        ]
        nonempty = [(p, values) for p, values in zip(phases, data) if len(values)]

        if nonempty:
            labels, values = zip(*nonempty)
            ax.boxplot(values, labels=labels, showfliers=False)
            ax.tick_params(axis="x", rotation=25)
        ax.set_title(SENSOR_LABELS[sensor])
        ax.set_ylabel("ADC Reading")
        ax.grid(True, axis="y", alpha=0.25)

    fig.suptitle("LDR ADC Distribution by Test Phase", fontsize=14)
    fig.tight_layout()
    fig.savefig(output_dir / "ldr_phase_boxplot.png", dpi=160)

    if show:
        plt.show()
    plt.close(fig)


def plot_phase_means(df: pd.DataFrame, output_dir: Path, show: bool):
    means = df.groupby("phase", observed=False)[SENSORS].mean()
    means = means.dropna(how="all")

    ax = means.plot(kind="bar", figsize=(13, 6), width=0.8)
    ax.set_title("Mean ADC Reading per LDR in Each Test Phase")
    ax.set_xlabel("Test Phase")
    ax.set_ylabel("Mean ADC Reading")
    ax.tick_params(axis="x", rotation=20)
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(
        [SENSOR_LABELS[s] for s in SENSORS],
        title="Sensor",
        ncol=4,
    )
    fig = ax.get_figure()
    fig.tight_layout()
    fig.savefig(output_dir / "ldr_phase_means.png", dpi=160)

    if show:
        plt.show()
    plt.close(fig)


def print_summary(df: pd.DataFrame):
    print("\n=== Mean ADC reading by phase ===")
    summary = df.groupby("phase", observed=False)[SENSORS].agg(
        ["count", "mean", "std", "min", "max"]
    )
    print(summary.round(2).to_string())

    if "DARK" in df["phase"].astype(str).unique():
        dark_means = df.loc[
            df["phase"].astype(str) == "DARK", SENSORS
        ].mean()

        print("\n=== Mean change from DARK baseline ===")
        for phase in PHASE_ORDER:
            phase_df = df[df["phase"].astype(str) == phase]
            if phase == "DARK" or phase_df.empty:
                continue
            phase_means = phase_df[SENSORS].mean()
            delta = phase_means - dark_means
            print(f"\n{phase}:")
            for sensor in SENSORS:
                print(
                    f"  {SENSOR_LABELS[sensor]:10s}: "
                    f"{delta[sensor]:+8.2f} ADC"
                )


def main():
    parser = argparse.ArgumentParser(
        description="Plot the four LDR channels from an ESP32 CSV log."
    )
    parser.add_argument(
        "--csv",
        required=True,
        help="CSV file generated by ldr_logger.py",
    )
    parser.add_argument(
        "--out",
        default="ldr_plots",
        help="Output folder for PNG plots (default: ldr_plots)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save plots without opening plot windows",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv)
    output_dir = Path(args.out)
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        df = load_data(csv_path)
        print(f"Loaded {len(df)} samples from: {csv_path}")
        print_summary(df)

        show = not args.no_show
        plot_timeseries(df, output_dir, show)
        plot_boxplot(df, output_dir, show)
        plot_phase_means(df, output_dir, show)

        print("\nPlots saved in:", output_dir.resolve())
        print(" - ldr_timeseries.png")
        print(" - ldr_phase_boxplot.png")
        print(" - ldr_phase_means.png")

    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
