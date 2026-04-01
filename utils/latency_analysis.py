"""
Latency analysis script for the stereo pipeline.

Reads per-node timing CSVs, filters to common message IDs, computes per-node
runtimes, and produces:
  1. A pipeline diagram showing rectify → (cuvslam || neustereo) with average latencies.
  2. Histograms showing the distribution of per-callback runtime for each node.

Usage:
    python -m stereo_pipeline_common.latency_analysis [--input-dir OUTPUT/LATENCY]
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd


def load_and_filter(input_dir: Path) -> pd.DataFrame:
    rectify_df = pd.read_csv(input_dir / 'rectify_timing.csv')
    cuvslam_df = pd.read_csv(input_dir / 'cuvslam_timing.csv')
    neustereo_df = pd.read_csv(input_dir / 'neustereo_timing.csv')

    common_ids = (
        set(rectify_df['message_id'])
        & set(cuvslam_df['message_id'])
        & set(neustereo_df['message_id'])
    )

    df = pd.concat([
        rectify_df[rectify_df['message_id'].isin(common_ids)],
        cuvslam_df[cuvslam_df['message_id'].isin(common_ids)],
        neustereo_df[neustereo_df['message_id'].isin(common_ids)],
    ], ignore_index=True)

    df['runtime_ms'] = (df['pub_time_ns'] - df['recv_time_ns']) / 1e6
    return df


def compute_stats(df: pd.DataFrame) -> pd.DataFrame:
    stats = df.groupby('node')['runtime_ms'].agg(['mean', 'std', 'min', 'max', 'median', 'count'])
    stats = stats.rename(columns={'mean': 'avg_ms', 'std': 'std_ms', 'min': 'min_ms',
                                  'max': 'max_ms', 'median': 'median_ms', 'count': 'n'})
    return stats


def plot_pipeline_diagram(stats: pd.DataFrame, output_dir: Path):
    """
    Draw a horizontal bar diagram:
      [  rectify  ] --+--> [  cuvslam   ]
                      +--> [  neustereo  ]
    Bar widths proportional to average runtime.
    """
    fig, ax = plt.subplots(figsize=(12, 4))

    colors = {'rectify': '#4C72B0', 'cuvslam': '#55A868', 'neustereo': '#C44E52'}
    rect_avg = stats.loc['rectify', 'avg_ms']
    cuv_avg = stats.loc['cuvslam', 'avg_ms']
    neu_avg = stats.loc['neustereo', 'avg_ms']

    bar_height = 0.6
    gap = 0.3

    y_cuv = 1.0
    y_neu = -0.2
    y_rect = (y_cuv + y_neu) / 2

    # Rectify bar
    ax.barh(y_rect, rect_avg, height=bar_height, color=colors['rectify'],
            edgecolor='black', linewidth=0.8, zorder=3)
    ax.text(rect_avg / 2, y_rect, f"rectify\n{rect_avg:.2f} ms",
            ha='center', va='center', fontweight='bold', fontsize=10, color='white', zorder=4)

    fork_x = rect_avg + gap

    # Cuvslam bar
    ax.barh(y_cuv, cuv_avg, left=fork_x, height=bar_height, color=colors['cuvslam'],
            edgecolor='black', linewidth=0.8, zorder=3)
    ax.text(fork_x + cuv_avg / 2, y_cuv, f"cuvslam\n{cuv_avg:.2f} ms",
            ha='center', va='center', fontweight='bold', fontsize=10, color='white', zorder=4)

    # Neustereo bar
    ax.barh(y_neu, neu_avg, left=fork_x, height=bar_height, color=colors['neustereo'],
            edgecolor='black', linewidth=0.8, zorder=3)
    ax.text(fork_x + neu_avg / 2, y_neu, f"neustereo\n{neu_avg:.2f} ms",
            ha='center', va='center', fontweight='bold', fontsize=10, color='white', zorder=4)

    # Fork arrows from rectify output to cuvslam and neustereo
    arrow_props = dict(arrowstyle='->', color='black', lw=1.5)
    ax.annotate('', xy=(fork_x, y_cuv), xytext=(rect_avg, y_rect), arrowprops=arrow_props)
    ax.annotate('', xy=(fork_x, y_neu), xytext=(rect_avg, y_rect), arrowprops=arrow_props)

    # End-to-end annotation
    parallel_max = max(cuv_avg, neu_avg)
    e2e = rect_avg + parallel_max
    ax.axvline(x=fork_x + parallel_max, color='gray', linestyle='--', linewidth=1, zorder=1)
    ax.text(fork_x + parallel_max + 0.3, y_cuv + 0.5,
            f"end-to-end avg: {e2e:.2f} ms\n({1000/e2e:.1f} Hz)",
            fontsize=10, va='bottom', color='gray')

    ax.set_xlim(-0.5, fork_x + parallel_max + e2e * 0.3)
    ax.set_ylim(-1.0, 2.0)
    ax.set_xlabel('Time (ms)')
    ax.set_yticks([])
    ax.set_title('Pipeline Latency Diagram (average per frame)')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)

    fig.tight_layout()
    fig.savefig(output_dir / 'pipeline_diagram.png', dpi=150)
    print(f"Saved pipeline diagram to {output_dir / 'pipeline_diagram.png'}")
    plt.close(fig)


def plot_runtime_distributions(df: pd.DataFrame, output_dir: Path):
    """Histograms of per-callback runtime for each node."""
    nodes = ['rectify', 'cuvslam', 'neustereo']
    colors = {'rectify': '#4C72B0', 'cuvslam': '#55A868', 'neustereo': '#C44E52'}

    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=False)

    for ax, node in zip(axes, nodes):
        runtimes = df[df['node'] == node]['runtime_ms']
        ax.hist(runtimes, bins=40, color=colors[node], edgecolor='black', linewidth=0.5, alpha=0.85)
        ax.axvline(runtimes.mean(), color='red', linestyle='--', linewidth=1.5,
                   label=f'mean: {runtimes.mean():.2f} ms')
        ax.axvline(runtimes.median(), color='orange', linestyle=':', linewidth=1.5,
                   label=f'median: {runtimes.median():.2f} ms')
        ax.set_title(f'{node} runtime distribution')
        ax.set_xlabel('Runtime (ms)')
        ax.set_ylabel('Count')
        ax.legend(fontsize=8)

    fig.suptitle('Per-Node Callback Runtime Distributions', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(output_dir / 'runtime_distributions.png', dpi=150, bbox_inches='tight')
    print(f"Saved runtime distributions to {output_dir / 'runtime_distributions.png'}")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='Stereo pipeline latency analysis')
    default_dir = Path(__file__).resolve().parents[1] / 'output' / 'latency'
    parser.add_argument('--input-dir', type=Path, default=default_dir,
                        help='Directory containing the timing CSVs')
    args = parser.parse_args()

    input_dir = args.input_dir
    if not input_dir.exists():
        print(f"Error: input directory {input_dir} does not exist")
        return

    print(f"Loading timing data from {input_dir}")
    df = load_and_filter(input_dir)
    n_common = df['message_id'].nunique()
    print(f"Found {n_common} frames with timing data from all 3 nodes")

    if n_common == 0:
        print("No common message IDs found across all 3 CSVs. Nothing to plot.")
        return

    stats = compute_stats(df)
    print("\n--- Per-Node Runtime Stats (ms) ---")
    print(stats.to_string())
    print()

    plot_pipeline_diagram(stats, input_dir)
    plot_runtime_distributions(df, input_dir)

    stats.to_csv(input_dir / 'runtime_stats.csv')
    print(f"Saved runtime stats to {input_dir / 'runtime_stats.csv'}")


if __name__ == '__main__':
    main()
