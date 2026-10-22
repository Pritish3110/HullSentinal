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
    """Dataflow of the two-robot stack.

    Boxes are laid out on a 4 x 2 lattice with 10-unit horizontal gaps so the topic
    labels fit between blocks without masking a border, and every feedback path is an
    explicit polyline routed through an empty corridor.
    """
    fig, ax = plt.subplots(figsize=(DCOL, 2.85))
    ax.set_xlim(0, 106); ax.set_ylim(5.0, 56); ax.axis('off')

    BW, BH = 17.0, 14.0
    TOP_Y, BOT_Y = 32.0, 11.0
    XS = [1.0, 28.0, 55.0, 82.0]
    FEED = '#9E2A2B'

    def box(x, y, title, lines, fc=PANEL, ec=EDGE):
        ax.add_patch(FancyBboxPatch((x, y), BW, BH,
                     boxstyle='round,pad=0.25,rounding_size=1.0',
                     fc=fc, ec=ec, lw=0.9, zorder=3))
        ax.text(x + BW / 2, y + BH - 3.0, title, ha='center', va='center',
                fontsize=7.3, fontweight='bold', color=NAVY, zorder=4)
        for i, ln in enumerate(lines):
            ax.text(x + BW / 2, y + BH - 6.6 - i * 3.0, ln, ha='center', va='center',
                    fontsize=6.1, color=MUTED, zorder=4)

    def harrow(x0, x1, y, label, fs=5.4):
        ax.add_patch(FancyArrowPatch((x0, y), (x1, y), arrowstyle='-|>',
                     mutation_scale=7.0, lw=0.85, color=DEEP, zorder=5))
        ax.text((x0 + x1) / 2, y + 1.6, label, ha='center', va='bottom',
                fontsize=fs, color=DEEP, style='italic', zorder=6,
                bbox=dict(fc='white', ec='none', pad=0.6))

    def varrow(x, y0, y1, label, lx, ly, color=DEEP, fs=5.4, ha='left'):
        ax.add_patch(FancyArrowPatch((x, y0), (x, y1), arrowstyle='-|>',
                     mutation_scale=7.0, lw=0.85, color=color, zorder=5))
        ax.text(lx, ly, label, ha=ha, va='center', fontsize=fs, color=color,
                style='italic', zorder=6, bbox=dict(fc='white', ec='none', pad=0.6))

    def route(pts, label, lxy, color=FEED, fs=5.4):
        xs = [q[0] for q in pts]; ys = [q[1] for q in pts]
        ax.plot(xs, ys, color=color, lw=0.85, zorder=5,
                solid_joinstyle='round', solid_capstyle='butt')
        ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle='-|>',
                     mutation_scale=7.0, lw=0.85, color=color, zorder=5))
        ax.text(lxy[0], lxy[1], label, ha='center', va='center', fontsize=fs,
                color=color, style='italic', zorder=6,
                bbox=dict(fc='white', ec='none', pad=0.6))

    # Cognition-layer enclosure, drawn under the blocks.
    ax.add_patch(FancyBboxPatch((25.5, 8.0), 49.0, 41.5,
                 boxstyle='round,pad=0.3,rounding_size=1.1',
                 fc='#FBFDFE', ec=DEEP, lw=1.1, ls=(0, (4, 3)), zorder=1))
    ax.text(50.0, 47.6, 'COGNITION LAYER  (contribution of this work)',
            ha='center', va='center', fontsize=6.8, fontweight='bold',
            color=DEEP, zorder=2)

    box(XS[0], TOP_Y, 'Simulation Engine',
        ['Kinematics, 14 defects', 'Sensor FOV raycast',
         r'Growth $\gamma_\tau$, cavitation'], fc='#E9F1F6')
    box(XS[1], TOP_Y, 'Belief Map',
        [r'$\langle S, C, \tau, \ell \rangle$ per defect', 'SQLite lifecycle',
         'Recursive fusion'])
    box(XS[2], TOP_Y, 'Utility Scorer',
        [r'Ranks by $U(d)$', r'A* geodesic $g_{A^*}$', 'Risk weighting'])
    box(XS[3], TOP_Y, 'Task Executive',
        ['Cleaner state machine', 'Priority context stack', 'Preempt and resume'],
        fc='#E9F1F6')

    box(XS[1], BOT_Y, 'Verification Monitor',
        ['Rescan handshake', r'residual $>\theta_v$:', 'escalate and requeue'])
    box(XS[2], BOT_Y, 'Lane Coordinator',
        [r'Gate: $\ell < \ell_{\mathrm{map}}$', 'Emergency override'])
    box(XS[3], BOT_Y, 'Robot Controllers',
        ['Serpentine mapper', 'A* crawler pursuit', 'Barrier and geofence'],
        fc='#E9F1F6')

    MID_Y = TOP_Y + BH / 2          # 39.0
    harrow(18.5, 27.5, MID_Y, '/defect_detections')
    harrow(45.5, 54.5, MID_Y, r'$\langle S, C \rangle$', fs=6.0)
    harrow(72.5, 81.5, MID_Y, '/task_queue')

    varrow(32.0, 25.3, 31.7, '/belief_updates', 33.2, 27.3)
    varrow(63.5, 25.3, 31.7, 'eligibility', 64.7, 27.0)
    varrow(90.0, 31.7, 25.3, '/target_goal', 91.2, 28.5)

    # Executive -> Verification, through the inter-row corridor.
    route([(86.0, 31.7), (86.0, 29.3), (41.0, 29.3), (41.0, 25.3)],
          '/task_complete', (50.0, 30.4))
    # Verification -> Simulation, around the left of the cognition layer.
    route([(27.7, 18.0), (7.0, 18.0), (7.0, 31.7)],
          '/rescan_request', (16.5, 19.3))
    # Controllers -> Simulation actuation, over the top of the figure.
    route([(99.2, 18.0), (103.0, 18.0), (103.0, 53.0), (12.0, 53.0), (12.0, 46.3)],
          '/cmd_vel, /pose', (56.0, 54.1), color=MID)

    fig.savefig(os.path.join(OUT, 'fig1_architecture.png'))
    plt.close(fig)


# ---------------------------------------------------------------- Fig. 2
def fig_lifecycle():
    """Per-defect belief lifecycle, drawn in greyscale for print fidelity.

    Forward transitions are solid; the two exception paths (requeue after a failed
    verification, and emergency preemption) are dashed and routed clear of every box.
    """
    fig, ax = plt.subplots(figsize=(5.40, 1.28))
    ax.set_xlim(0, 100); ax.set_ylim(7.6, 45); ax.axis('off')

    BW, BH, BY = 17.0, 10.0, 20.0
    XS = [0.0, 20.75, 41.5, 62.25, 83.0]
    names = ['UNDETECTED', 'DETECTED', 'QUEUED', 'CLEANING', 'VERIFIED']
    fills = ['#FFFFFF', '#FFFFFF', '#FFFFFF', '#EDEDED', '#D9D9D9']
    cx = [x + BW / 2 for x in XS]

    for x, n, fc in zip(XS, names, fills):
        ax.add_patch(FancyBboxPatch((x, BY), BW, BH,
                     boxstyle='round,pad=0.25,rounding_size=1.6',
                     fc=fc, ec='#1A1A1A', lw=0.9, zorder=3))
        ax.text(x + BW / 2, BY + BH / 2, n, ha='center', va='center',
                fontsize=6.4, fontweight='bold', color='#1A1A1A', zorder=4)

    def fwd(i, label):
        x0, x1 = XS[i] + BW + 0.45, XS[i + 1] - 0.45
        ax.add_patch(FancyArrowPatch((x0, BY + BH / 2), (x1, BY + BH / 2),
                     arrowstyle='-|>', mutation_scale=6.5, lw=0.9,
                     color='#1A1A1A', zorder=5))
        ax.text((x0 + x1) / 2, BY + BH + 1.6, label, ha='center', va='bottom',
                fontsize=5.5, color='#1A1A1A', style='italic', zorder=6)

    fwd(0, 'FOV hit')
    fwd(1, r'$U(d)$ ranked')
    fwd(2, 'dispatch')
    fwd(3, r'residual $\leq\theta_v$')

    def dashed(pts, label, lxy, va='center'):
        xs = [q[0] for q in pts]; ys = [q[1] for q in pts]
        ax.plot(xs, ys, color='#1A1A1A', lw=0.85, ls=(0, (3.2, 2.2)), zorder=5,
                solid_joinstyle='round')
        ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle='-|>',
                     mutation_scale=6.5, lw=0.85, color='#1A1A1A', zorder=5))
        ax.text(lxy[0], lxy[1], label, ha='center', va=va, fontsize=5.5,
                color='#1A1A1A', style='italic', zorder=6)

    # Failed verification returns the defect to the queue.
    dashed([(cx[3], BY), (cx[3], 12.5), (cx[2], 12.5), (cx[2], BY - 0.4)],
           r'residual $>\theta_v$:  RE-QUEUED', ((cx[2] + cx[3]) / 2, 10.8), va='top')
    # A defect crossing the critical threshold preempts whatever is running.
    dashed([(cx[1], BY + BH), (cx[1], 38.5), (cx[3], 38.5), (cx[3], BY + BH + 0.4)],
           r'$S \geq \theta_c$:  emergency preemption (context stack)',
           ((cx[1] + cx[3]) / 2, 39.6), va='bottom')

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
