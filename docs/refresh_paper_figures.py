#!/usr/bin/env python3
"""
Re-embeds the current docs/paper_figures/*.png into the IEEE manuscript.

The manuscript body is edited directly in Word; only its figures are generated.
This script swaps the image bytes inside the .docx and rewrites each drawing's
extent so every figure is drawn at its native aspect ratio (a plain byte swap
would stretch the image to the old extent).

Workflow:
    python3 docs/paper_figures/make_figures.py      # regenerate the PNGs
    python3 docs/refresh_paper_figures.py           # push them into the .docx
    soffice --headless --convert-to pdf docs/HullSentinal_IEEE_Paper.docx

Figures are matched to the embedded images by document order, so the order of
FIGURE_ORDER below must match the order the figures appear in the manuscript.
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile

from PIL import Image

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
DOCX = os.path.join(ROOT, 'docs', 'HullSentinal_IEEE_Paper.docx')
FIGS = os.path.join(ROOT, 'docs', 'paper_figures')
EMU_IN = 914400

# Document order of the figures, with the width each is placed at (inches).
# 7.25 in is the full text width; 3.521 in is one column.
FIGURE_ORDER = [
    ('fig1_architecture.png', 7.250),
    ('fig2_lifecycle.png',    5.400),
    ('fig3_world.png',        7.250),
    ('fig4_stages.png',       7.250),
    ('fig5_order.png',        3.521),
    ('fig6_correlation.png',  3.521),
    ('fig7_tradeoff.png',     3.521),
]


def unpack(docx, dest):
    with zipfile.ZipFile(docx) as z:
        z.extractall(dest)


def repack(src, docx):
    tmp = docx + '.tmp'
    with zipfile.ZipFile(tmp, 'w', zipfile.ZIP_DEFLATED) as z:
        for base, _, files in os.walk(src):
            for f in files:
                full = os.path.join(base, f)
                z.write(full, os.path.relpath(full, src))
    shutil.move(tmp, docx)


def main():
    if not os.path.exists(DOCX):
        sys.exit(f'manuscript not found: {DOCX}')

    work = tempfile.mkdtemp(prefix='hullpaper_')
    unpack(DOCX, work)

    doc_path = os.path.join(work, 'word', 'document.xml')
    rels_path = os.path.join(work, 'word', '_rels', 'document.xml.rels')
    with open(doc_path, encoding='utf-8') as f:
        doc = f.read()
    with open(rels_path, encoding='utf-8') as f:
        rid2tgt = dict(re.findall(r'Id="(rId\d+)"[^>]*Target="([^"]+)"', f.read()))

    state = {'i': 0}

    def rewrite(m):
        block = m.group(0)
        rid = re.search(r'r:embed="(rId\d+)"', block)
        if not rid:
            return block
        idx = state['i']
        state['i'] += 1
        if idx >= len(FIGURE_ORDER):
            print(f'  warning: drawing {idx + 1} has no entry in FIGURE_ORDER; left as is')
            return block

        fig, width_in = FIGURE_ORDER[idx]
        src = os.path.join(FIGS, fig)
        if not os.path.exists(src):
            print(f'  warning: {fig} missing; left as is')
            return block

        with Image.open(src) as im:
            pw, ph = im.size
        cx = int(round(width_in * EMU_IN))
        cy = int(round(cx * ph / pw))
        block = re.sub(r'(<wp:extent cx=")\d+(" cy=")\d+(")',
                       r'\g<1>%d\g<2>%d\g<3>' % (cx, cy), block)
        block = re.sub(r'(<a:ext cx=")\d+(" cy=")\d+(")',
                       r'\g<1>%d\g<2>%d\g<3>' % (cx, cy), block)

        media = rid2tgt[rid.group(1)].split('/')[-1]
        shutil.copyfile(src, os.path.join(work, 'word', 'media', media))
        print(f'  Fig. {idx + 1}: {fig:26s} {pw}x{ph}px -> '
              f'{cx / EMU_IN:.3f} x {cy / EMU_IN:.3f} in')
        return block

    doc = re.sub(r'<w:drawing>.*?</w:drawing>', rewrite, doc, flags=re.S)
    with open(doc_path, 'w', encoding='utf-8') as f:
        f.write(doc)

    repack(work, DOCX)
    shutil.rmtree(work, ignore_errors=True)
    print(f'\nupdated {DOCX}')


if __name__ == '__main__':
    main()
