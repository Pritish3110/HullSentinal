#!/usr/bin/env python3
"""
Generates the publication figures for the HullSentinal IEEE paper.

All figures are sized to the IEEE two-column format (3.4 in single column,
7.16 in double column) and typeset in Times New Roman so they match the
body text of the manuscript.

Usage:  python3 docs/paper_figures/make_figures.py
"""

import os
import csv
import math
import statistics as st

import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
from PIL import Image

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
OUT = os.path.join(ROOT, 'docs', 'paper_figures')
DOCS = os.path.join(ROOT, 'docs')
CFG = os.path.join(ROOT, 'ws', 'src', 'hull_bringup', 'config', 'hullsentinal_v2.yaml')
os.makedirs(OUT, exist_ok=True)

COL, DCOL = 3.4, 7.16
DPI = 400

# Palette carried over from the project deck so paper and slides read as one system.
NAVY = '#21295C'
DEEP = '#065A82'
MID = '#1C7293'
TEAL = '#0A9396'
CRIMSON = '#C9283E'
INK = '#1B1B1B'
MUTED = '#5B6B75'
PANEL = '#F4F8FA'
EDGE = '#9FC4DA'

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Nimbus Roman', 'Liberation Serif'],
    'font.size': 8,
    'axes.labelsize': 8,
    'axes.titlesize': 8.5,
    'xtick.labelsize': 7.5,
    'ytick.labelsize': 7.5,
    'legend.fontsize': 7,
    'axes.linewidth': 0.7,
    'xtick.major.width': 0.7,
    'ytick.major.width': 0.7,
    'figure.dpi': DPI,
    'savefig.dpi': DPI,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.02,
})

SCEN = ['nominal', 'swarm_scaling', 'obstacles', 'high_sensor_noise',
        'agent_failure', 'marine_current']
SCEN_LBL = ['Nominal', 'Swarm\nscaling', 'Obstacles', 'High sensor\nnoise',
            'Agent\nfailure', 'Marine\ncurrent']


def load_rows():
    with open(os.path.join(DOCS, 'benchmark_results.csv')) as f:
        return list(csv.DictReader(f))


def agg(rows, scen, system, key, scale=1.0):
    v = [float(x[key]) * scale for x in rows
         if x['scenario'] == scen and x['system'] == system]
    return st.mean(v), (st.stdev(v) if len(v) > 1 else 0.0)


# ---------------------------------------------------------------- Fig. 1
def fig_architecture():
    """Dataflow of the two-robot stack. Feedback paths are routed as explicit
    polylines through empty corridors so no arrow crosses a block."""
    fig, ax = plt.subplots(figsize=(DCOL, 2.95))
    ax.set_xlim(0, 100); ax.set_ylim(0, 46); ax.axis('off')

    TOP_Y, TOP_H = 28.0, 12.0
    BOT_Y, BOT_H = 11.5, 11.0

    def box(x, y, w, h, title, lines, fc=PANEL, ec=EDGE, lw=0.9):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
                     boxstyle='round,pad=0.3,rounding_size=1.0',
                     fc=fc, ec=ec, lw=lw, zorder=3))
        ax.text(x + w / 2, y + h - 2.6, title, ha='center', va='center',
                fontsize=7.4, fontweight='bold', color=NAVY, zorder=4)
        for i, ln in enumerate(lines):
            ax.text(x + w / 2, y + h - 5.8 - i * 2.8, ln, ha='center', va='center',
                    fontsize=6.2, color=MUTED, zorder=4)
        return (x, y, w, h)

    def straight(p, q, label='', color=DEEP, lpos='above', fs=5.8, rad=0.0):
        ax.add_patch(FancyArrowPatch(p, q, connectionstyle=f'arc3,rad={rad}',
                     arrowstyle='-|>', mutation_scale=7.0, lw=0.85,
                     color=color, zorder=5))
        if not label:
            return
        mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
        off = {'above': (0, 1.3, 'center', 'bottom'),
               'below': (0, -1.3, 'center', 'top'),
               'left':  (-1.0, 0, 'right', 'center'),
               'right': (1.0, 0, 'left', 'center')}[lpos]
        ax.text(mx + off[0], my + off[1], label, ha=off[2], va=off[3],
                fontsize=fs, color=color, style='italic', zorder=6,
                bbox=dict(fc='white', ec='none', pad=0.8))

    def polyline(pts, label='', lxy=None, color=CRIMSON, fs=5.8):
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        ax.plot(xs, ys, color=color, lw=0.85, zorder=5,
                solid_joinstyle='round', solid_capstyle='round')
        ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle='-|>',
                     mutation_scale=7.0, lw=0.85, color=color, zorder=5))
        if label and lxy:
            ax.text(lxy[0], lxy[1], label, ha='center', va='center',
                    fontsize=fs, color=color, style='italic', zorder=6,
                    bbox=dict(fc='white', ec='none', pad=0.9))

    # Cognition-layer container drawn first so blocks sit on top of it.
    ax.add_patch(FancyBboxPatch((21.5, 9.6), 41.5, 33.7,
                 boxstyle='round,pad=0.4,rounding_size=1.2',
                 fc='#FBFDFE', ec=DEEP, lw=1.2, ls=(0, (4, 3)), zorder=1))
    ax.text(42.2, 41.8, 'COGNITION LAYER  (contribution of this work)',
            ha='center', va='center', fontsize=6.9, fontweight='bold',
            color=DEEP, zorder=2)

    box(1, TOP_Y, 18.5, TOP_H, 'Simulation Engine',
        ['Kinematics, 14 defects', 'Sensor FOV raycast',
         r'Growth $\gamma_\tau$, cavitation'], fc='#E9F1F6')
    box(23.5, TOP_Y, 17.5, TOP_H, 'Belief Map',
        [r'$\langle S, C, \tau, \ell \rangle$ per defect', 'SQLite lifecycle',
         'Recursive fusion'])
    box(44.5, TOP_Y, 17.5, TOP_H, 'Utility Scorer',
        [r'Ranks by $U(d)$', r'A* geodesic $g_{A^*}$', 'Risk weighting'])
    box(66.5, TOP_Y, 18.5, TOP_H, 'Task Executive',
        ['Cleaner state machine', 'Priority context stack', 'Preempt and resume'],
        fc='#E9F1F6')

    box(23.5, BOT_Y, 17.5, BOT_H, 'Verification Monitor',
        ['Rescan handshake', r'residual $>\theta_v$: requeue'])
    box(44.5, BOT_Y, 17.5, BOT_H, 'Lane Coordinator',
        [r'Gate: $\ell < \ell_{\mathrm{map}}$', 'Emergency override'])
    box(66.5, BOT_Y, 18.5, BOT_H, 'Robot Controllers',
        ['Serpentine mapper', 'A* crawler pursuit', 'Barrier and geofence'],
        fc='#E9F1F6')

    # Forward chain along the top row.
    straight((19.7, 34.0), (23.3, 34.0), '/defect_detections', lpos='above')
    straight((41.2, 34.0), (44.3, 34.0), r'$\langle S,C\rangle$', lpos='above')
    straight((62.2, 34.0), (66.3, 34.0), '/task_queue', lpos='above')

    # Vertical couplings between aligned blocks.
    straight((56.5, 22.7), (56.5, 27.8), '', lpos='right', fs=5.6)
    ax.text(57.4, 23.5, 'eligibility', ha='left', va='center', fontsize=5.6,
            color=DEEP, style='italic', zorder=6)
    straight((75.75, 27.8), (75.75, 22.7), '/target_goal', lpos='right', fs=5.6)
    straight((32.25, 22.7), (32.25, 27.8), '/belief_updates', lpos='left', fs=5.6)

    # Executive -> Verification, routed through the empty corridor between rows.
    polyline([(70.0, 27.8), (70.0, 25.4), (37.0, 25.4), (37.0, 22.9)],
             '/task_complete', (48.5, 26.6))

    # Verification -> Simulation rescan, routed below the whole figure.
    polyline([(23.3, 17.0), (10.25, 17.0), (10.25, 27.6)],
             '/rescan_request', (16.2, 18.4))

    # Controllers -> Simulation actuation, routed above the top row.
    polyline([(85.3, 17.0), (92.5, 17.0), (92.5, 44.2), (10.25, 44.2),
              (10.25, 40.2)], '/cmd_vel, /pose', (50.0, 45.0), color=MID)

    fig.savefig(os.path.join(OUT, 'fig1_architecture.png'))
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 2
def fig_lifecycle():
    """Per-defect belief lifecycle: forward chain, requeue loop, preempt fast path."""
    fig, ax = plt.subplots(figsize=(DCOL, 1.62))
    ax.set_xlim(0, 100); ax.set_ylim(0, 48); ax.axis('off')

    W, H, Y = 17.0, 10.0, 22.0
    xs = [1.0, 21.5, 42.0, 62.5, 83.0]
    names = ['UNDETECTED', 'DETECTED', 'QUEUED', 'CLEANING', 'VERIFIED']
    cols = ['#9AA3AA', '#E3B40F', '#E07B29', '#C9283E', TEAL]
    tcol = ['white', INK, 'white', 'white', 'white']

    for x, n, c, tc in zip(xs, names, cols, tcol):
        ax.add_patch(FancyBboxPatch((x, Y), W, H,
                     boxstyle='round,pad=0.3,rounding_size=2.0',
                     fc=c, ec='none', zorder=3))
        ax.text(x + W / 2, Y + H / 2, n, ha='center', va='center',
                fontsize=6.5, fontweight='bold', color=tc, zorder=4)

    def edge(i, j, label):
        p0 = (xs[i] + W + 0.4, Y + H / 2)
        p1 = (xs[j] - 0.4, Y + H / 2)
        ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle='-|>', mutation_scale=7.0,
                     lw=0.9, color=DEEP, zorder=5))
        ax.text((p0[0] + p1[0]) / 2, Y + H + 1.6, label, ha='center', va='bottom',
                fontsize=5.5, color=DEEP, style='italic', zorder=6)

    edge(0, 1, 'FOV hit')
    edge(1, 2, r'$U(d)$ ranked')
    edge(2, 3, 'dispatch')
    edge(3, 4, r'residual $\leq \theta_v$')

    # Requeue loop, routed under the chain.
    ax.plot([71.0, 71.0, 50.5, 50.5], [Y, 11.0, 11.0, Y - 0.4],
            color=CRIMSON, lw=0.9, zorder=5, solid_joinstyle='round')
    ax.add_patch(FancyArrowPatch((50.5, 13.0), (50.5, Y - 0.4), arrowstyle='-|>',
                 mutation_scale=7.0, lw=0.9, color=CRIMSON, zorder=5))
    ax.text(60.75, 9.2, r'residual $> \theta_v$:  RE-QUEUED', ha='center', va='top',
            fontsize=5.5, color=CRIMSON, style='italic', zorder=6)

    # Emergency fast path, routed over the chain.
    ax.plot([30.0, 30.0, 71.0, 71.0], [Y + H, 41.0, 41.0, Y + H + 0.4],
            color='#8E1F2F', lw=0.9, zorder=5, solid_joinstyle='round')
    ax.add_patch(FancyArrowPatch((71.0, 43.0), (71.0, Y + H + 0.4), arrowstyle='-|>',
                 mutation_scale=7.0, lw=0.9, color='#8E1F2F', zorder=5))
    ax.text(50.5, 42.6, r'$S \geq \theta_c$:  emergency preemption (context stack)',
            ha='center', va='bottom', fontsize=5.5, color='#8E1F2F',
            style='italic', zorder=6)

    fig.savefig(os.path.join(OUT, 'fig2_lifecycle.png'))
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 3
def fig_world():
    with open(CFG) as f:
        cfg = yaml.safe_load(f)
    W = float(cfg['world']['hull_width']); H = float(cfg['world']['hull_height'])

    fig, ax = plt.subplots(figsize=(DCOL, 2.52))
    ax.add_patch(mpatches.Rectangle((0, 0), W, H, fc='#EEF4F8', ec=NAVY, lw=1.1))

    for y in (3.0, 5.0):
        ax.axhline(y, color=EDGE, ls=(0, (5, 4)), lw=0.7)
    for i, y in enumerate((2.0, 4.0, 6.0)):
        ax.text(-0.75, y, f'L{i+1}', ha='center', va='center', fontsize=7,
                color=DEEP, fontweight='bold')

    for o in cfg['static_obstacles']:
        ax.add_patch(mpatches.Rectangle(
            (o['x'] - o['width'] / 2, o['y'] - o['height'] / 2),
            o['width'], o['height'], fc='#8C98A4', ec='#5B6B75', lw=0.6, zorder=3))
        ax.add_patch(mpatches.Rectangle(
            (o['x'] - o['width'] / 2 - 0.55, o['y'] - o['height'] / 2 - 0.55),
            o['width'] + 1.10, o['height'] + 1.10,
            fc='none', ec='#B9454F', lw=0.5, ls=(0, (2, 2)), zorder=2))

    mark = {'crack': ('s', CRIMSON, 'Crack'), 'rust': ('o', '#C8761E', 'Corrosion'),
            'biofouling': ('^', '#2E8B57', 'Biofouling')}
    seen = set()
    for d in cfg['defects']:
        m, c, lbl = mark[d['type']]
        ax.scatter(d['x'], d['y'], marker=m, s=16 + 52 * d['severity'], c=c,
                   edgecolors='white', linewidths=0.5, zorder=5,
                   label=lbl if lbl not in seen else None)
        seen.add(lbl)

    ax.scatter([1.5], [2.0], marker='D', s=42, c=TEAL, edgecolors='white',
               linewidths=0.6, zorder=6, label='Mapper spawn')
    ax.scatter([1.5], [7.0], marker='D', s=42, c='#E8701A', edgecolors='white',
               linewidths=0.6, zorder=6, label='Cleaner spawn')

    ax.set_xlim(-1.6, W + 0.4); ax.set_ylim(-0.4, H + 0.4)
    ax.set_aspect('equal')
    ax.set_xlabel('Hull longitudinal axis $x$ (m)')
    ax.set_ylabel('$y$ (m)')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.26), ncol=5,
              frameon=False, handletextpad=0.3, columnspacing=1.1)
    for s in ax.spines.values():
        s.set_color('#AAB4BC')
    fig.savefig(os.path.join(OUT, 'fig3_world.png'))
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 4
def fig_stages():
    """2x2 composite of real simulator output captured during one mission."""
    files = [('screenshot_start.png', '(a) $t\\approx2$ s — sweep begins, all defects UNDETECTED'),
             ('screenshot_lane_coordination.png', '(b) Lane 1 released — cleaner authorised'),
             ('screenshot_emergency_interruption.png', '(c) Escalated crack preempts active task'),
             ('screenshot_end.png', '(d) Mission end — 14/14 VERIFIED_CLEAN')]
    fig, axes = plt.subplots(2, 2, figsize=(DCOL, 3.35))
    for ax, (fn, cap) in zip(axes.ravel(), files):
        img = Image.open(os.path.join(DOCS, fn))
        w, h = img.size
        img = img.crop((int(w * 0.055), int(h * 0.05), int(w * 0.985), int(h * 0.90)))
        ax.imshow(img)
        ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values():
            s.set_color('#AAB4BC'); s.set_linewidth(0.5)
        ax.set_title(cap, fontsize=6.0, color=INK, pad=2.4)
    fig.subplots_adjust(wspace=0.04, hspace=0.20)
    fig.savefig(os.path.join(OUT, 'fig4_stages.png'))
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 5/6
def _grouped(rows, key, scale, ylabel, fname, ylim=None, hline=None):
    fig, ax = plt.subplots(figsize=(COL, 2.18))
    xs = range(len(SCEN)); w = 0.37
    om = [agg(rows, s, 'prioritized', key, scale)[0] for s in SCEN]
    osd = [agg(rows, s, 'prioritized', key, scale)[1] for s in SCEN]
    nm = [agg(rows, s, 'naive_baseline', key, scale)[0] for s in SCEN]
    nsd = [agg(rows, s, 'naive_baseline', key, scale)[1] for s in SCEN]

    ax.bar([x - w / 2 for x in xs], om, w, yerr=osd, capsize=1.8,
           color=TEAL, edgecolor='#07696C', linewidth=0.5,
           error_kw=dict(elinewidth=0.6, ecolor=INK), label='HullSentinal V2')
    ax.bar([x + w / 2 for x in xs], nm, w, yerr=nsd, capsize=1.8,
           color='#D4495A', edgecolor='#9B2432', linewidth=0.5,
           error_kw=dict(elinewidth=0.6, ecolor=INK), label='Naive FIFO')
    if hline is not None:
        ax.axhline(hline, color=MUTED, lw=0.6, ls=':')
    ax.set_xticks(list(xs)); ax.set_xticklabels(SCEN_LBL, fontsize=6.0)
    ax.set_ylabel(ylabel)
    if ylim: ax.set_ylim(*ylim)
    ax.grid(axis='y', color='#D7DEE3', lw=0.5, ls='-')
    ax.set_axisbelow(True)
    ax.legend(frameon=False, loc='upper center', bbox_to_anchor=(0.5, 1.21), ncol=2)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    fig.savefig(os.path.join(OUT, fname))
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 7
def fig_tradeoff(rows):
    fig, ax = plt.subplots(figsize=(COL, 2.18))
    for system, c, m, lbl in (('prioritized', TEAL, 'o', 'HullSentinal V2'),
                              ('naive_baseline', '#D4495A', 's', 'Naive FIFO')):
        xs = [float(r['time_to_clean_sec']) for r in rows if r['system'] == system]
        ys = [float(r['order_correctness']) * 100 for r in rows if r['system'] == system]
        ax.scatter(xs, ys, s=13, c=c, marker=m, alpha=0.78, edgecolors='white',
                   linewidths=0.35, label=lbl, zorder=3)
    ax.set_xlabel('Mission time-to-clean (s)')
    ax.set_ylabel('Order correctness (%)')
    ax.grid(color='#D7DEE3', lw=0.5)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, loc='lower right')
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    fig.savefig(os.path.join(OUT, 'fig7_tradeoff.png'))
    plt.close(fig)


def main():
    rows = load_rows()
    fig_architecture()
    fig_lifecycle()
    fig_world()
    fig_stages()
    _grouped(rows, 'order_correctness', 100.0, 'Order correctness (%)',
             'fig5_order.png', ylim=(0, 112), hline=50)
    _grouped(rows, 'utility_rank_correlation', 1.0, r'Spearman $r_s$',
             'fig6_correlation.png', ylim=(-0.25, 1.12), hline=0)
    fig_tradeoff(rows)
    for f in sorted(os.listdir(OUT)):
        if f.endswith('.png'):
            print('  ', f, f'{os.path.getsize(os.path.join(OUT, f))/1024:.0f} KB')


if __name__ == '__main__':
    main()
