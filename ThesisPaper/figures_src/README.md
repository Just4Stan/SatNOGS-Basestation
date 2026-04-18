# Thesis figures — live editor

Source for the six figures referenced by the thesis body. Each figure has two
pipelines: a Mermaid-based one for block/flow diagrams (GUI-editable in the
browser), and a matplotlib script for data-driven plots.

## Workflow A — Mermaid (figs 1, 2, 4, 5, 6a)

1. Open `preview.html` directly in a browser — no server, no install.
2. Edit the Mermaid source in any textarea. Preview updates live.
3. Click **Export PNG** to download. Drop the PNG in `../images/figures/`
   renamed to match (e.g. `system_block_diagram.png`).
4. If the change is worth keeping in git, click **Download .mmd** and
   overwrite the matching file in `mermaid/`.

Export scale lives in the top toolbar. 3× ≈ thesis DPI; bump to 4–5 for
large figures that will span the text width.

## Workflow B — matplotlib (figs 3, 6b)

Programmatic figures: COBS byte layout, Smith chart. Edit the `.py`, rerun:

```
make all                  # regenerate all PNGs
python3 cobs_frame_diagram.py   # one at a time
```

Outputs land in `../images/figures/`. Palette and font constants live in
`_style.py`; change once, all figures update.

## KU Leuven style

Matches the FIIW template (`ThesisPaper/fiiw.sty`):

- `kulblueDark = #00407A` — primary / dark
- `kulblue    = #1D8DB0` — accent
- Sans-serif (approximates `cmbright`); matplotlib at 240 DPI

## File map

```
figures_src/
├── preview.html              ← live Mermaid editor (open in browser)
├── _style.py                 ← matplotlib palette + fonts
├── Makefile                  ← `make all` regenerates all matplotlib PNGs
├── mermaid/                  ← .mmd source files (git-friendly)
│   ├── system_block.mmd
│   ├── power_distribution.mmd
│   ├── station_dataflow.mmd
│   ├── field_deployment.mmd
│   ├── matching_network.mmd
│   └── cobs_frame.mmd        (packet-beta, experimental)
├── system_block_diagram.py           ← matplotlib fallbacks
├── power_distribution_diagram.py
├── cobs_frame_diagram.py             ← primary (no Mermaid equivalent)
├── station_dataflow_diagram.py
├── field_deployment_workflow.py
└── matching_network_annotated.py     ← schematic + Smith chart
```

## Tips

- If the preview stays blank, watch the status pill next to the card title —
  it flips to red on Mermaid syntax errors.
- For precise control (exact box positions, overlapping labels, Smith-chart
  data points), matplotlib is the right tool — stay in workflow B.
- For quick semantic edits ("add a node", "rename an arrow", "change colour"),
  stay in workflow A — Mermaid re-lays out automatically.
