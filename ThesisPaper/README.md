# Thesis Paper

LaTeX source for the Master's thesis:

> **"Design and Implementation of an Economical SatNOGS-Compatible Ground Station for the AetherSpace CubeSat"**
>
> Stan Coene & Robbe Lehaen -- KU Leuven, Campus Geel, ELO-ICT, 2025--2026
> Promotor: Prof. dr. ir. J. Vanhamel | Copromotor: Prof. dr. ir. V. De Smedt

---

## Setup

### Windows

1. **Install MiKTeX** (LaTeX distribution):
   - Download from https://miktex.org/download
   - Run the installer, select "Install for all users"
   - During install, set "Install missing packages on-the-fly" to **Yes**
   - MiKTeX will auto-download any packages it needs on first compile

2. **Install VS Code extensions**:
   - Open VS Code
   - Go to Extensions (`Ctrl+Shift+X`)
   - Search and install **LaTeX Workshop** by James Yu

3. **Open the project**:
   - File > Open Folder > navigate to `ThesisPaper/`
   - Open `thesis.tex`
   - Press `Ctrl+Alt+B` to build, or just save any `.tex` file (auto-builds)
   - Press `Ctrl+Alt+V` to open the PDF preview side-by-side

4. **First build will be slow** — MiKTeX downloads missing packages. Subsequent builds are fast (~5s).

### macOS

```bash
brew install --cask mactex
```

Restart terminal after install. Then install **LaTeX Workshop** in VS Code (same as above, but use `Cmd` instead of `Ctrl`).

### VS Code settings (both platforms)

Add to `.vscode/settings.json` (or user settings):

```json
{
  "latex-workshop.latex.autoBuild.run": "onSave",
  "latex-workshop.view.pdf.viewer": "tab",
  "latex-workshop.latex.clean.fileTypes": [
    "*.aux", "*.bbl", "*.blg", "*.fdb_latexmk", "*.fls",
    "*.lof", "*.log", "*.lot", "*.out", "*.toc", "*.synctex.gz",
    "*.bcf", "*.run.xml"
  ]
}
```

### Terminal build (optional)

```bash
cd ThesisPaper/
make          # build once
make watch    # auto-rebuild on file changes
make clean    # remove build artifacts
```

---

## Daily workflow

1. `git pull` before you start writing
2. Edit your chapter `.tex` files — PDF rebuilds on save
3. Put figures in `images/`
4. Add references to `bibliography.bib`
5. Commit and push when done

Keep **one sentence per line** in `.tex` files — makes git diffs readable.

---

## File structure

```
thesis.tex              Main document (compile this)
fiiw.sty                KU Leuven FIIW style (don't modify unless layout changes)
bibliography.bib        BibTeX references
draft.md                Full draft in markdown (reference, don't submit)

chapters/
  abstract-EN.tex       English abstract (LANGUAGE: English)
  abstract-NL.tex       Dutch samenvatting (LANGUAGE: Dutch)
  preface.tex           Voorwoord (Dutch or English, your choice)
  abbreviations.tex     Abbreviation/acronym list
  ch01_introduction.tex
  ...
  ch12_conclusion.tex
  appendix_a-e          Appendices

images/                 All figures, photos, diagrams
covers/                 KU Leuven cover pages (don't modify)
```

---

## Who writes what

| Owner | Chapters | Topic |
|-------|----------|-------|
| **Stan** | ch01–ch07, ch09–ch12, app A–C/E | Rotator, electronics, firmware, Pi, results |
| **Robbe** | ch08, appendix D | RF front-end, CC1200 HAT, SmartRF config |
| **Both** | ch09 (BOM), ch11 (discussion) | Cost analysis covers both subsystems |

Chapters are separate files — no merge conflicts if you stick to your own files.

---

## Quick LaTeX reference

### Structure
```latex
\chapter{Title}              % Chapter
\section{Title}              % Subchapter
\subsection{Title}           % Sub-subchapter
```

### Figures
```latex
\begin{figure}[H]
    \centering
    \includegraphics[width=0.8\textwidth]{my_image.png}
    \caption{Description.}
    \label{fig:my-image}
\end{figure}
```

### Tables
```latex
\begin{table}[H]
    \centering
    \begin{tabular}{lcc}
        \toprule
        Header & Col 1 & Col 2 \\
        \midrule
        Row 1 & data & data \\
        \bottomrule
    \end{tabular}
    \caption{Description.}
    \label{tab:my-table}
\end{table}
```

### Formatting
| Command | What it does |
|---------|-------------|
| `\textbf{bold}` | **bold** |
| `\textit{italic}` | *italic* |
| `\code{rotctld}` | inline code |
| `\SI{24}{\volt}` | 24 V (SI units) |
| `35\degree` | 35° |
| `\cite{key}` | [1] (citation) |
| `\ref{fig:name}` | Figure 3.1 |
| `\label{fig:name}` | Define a reference target |

### Math
```latex
Inline: $f = \frac{c}{\lambda}$
Display:
\begin{equation}
    P_r = P_t G_t G_r \left(\frac{\lambda}{4\pi d}\right)^2
\end{equation}
```

### Lists
```latex
\begin{itemize}
    \item First point
    \item Second point
\end{itemize}

\begin{enumerate}
    \item Step one
    \item Step two
\end{enumerate}
```

Use `\toprule`, `\midrule`, `\bottomrule` for tables (not `\hline`).
