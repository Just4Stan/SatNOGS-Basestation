# G10 — SatNOGS history, CubeSat context, AetherSpace

Date: 2026-04-18. Scope: historical/contextual claims in ch01, ch02, ch05, bibliography.bib.

## 1. SatNOGS origin (ch02 L101)

Claim: "SatNOGS originated at Hackerspace.gr in Athens during 2014 NASA Space Apps Challenge"

**VERIFIED.** Wikipedia/SatNOGS team page/Hackaday/Space Apps Challenge records all confirm: project was first proposed at Hackerspace.gr (Athens) during the 2014 International Space Apps Challenge (April 2014), under the "Virtual Ground Station App – Global Crowdsourcing of CubeSats" challenge.
Sources: `en.wikipedia.org/wiki/SatNOGS`, `satnogs.org/team/`, `2014.spaceappschallenge.org/project/satnogs/`.

## 2. Hackaday Prize + LSF funding (ch02 L101)

Claim: "Won that year's Hackaday Prize, funded the Libre Space Foundation"

**VERIFIED.** SatNOGS won 1st prize at the 2014 Hackaday Prize, announced 2014-11-13. The cash prize seeded the Libre Space Foundation.
Sources: `hackaday.com/2014/11/13/satnogs-wins-the-2014-hackaday-prize/`, `satnogs.org/2019/11/14/5yrs-hackadayprize-2/`.

## 3. Hackaday Prize $196,418 (bibliography L446–447)

Claim: "Hackaday Prize 2014 SatNOGS Grand Prize $196,418"

**PARTIALLY VERIFIED / MINOR UNCERTAINTY.** Multiple secondary sources cite $196,418; the Hackaday Prize 2014 rewards page mentions the "cash equivalent" option was $198,418. Exact figure varies by source ($196k vs $198k). The $196,418 figure is in wide circulation but may be off by ~$2k. Not load-bearing — consider softening to "approximately $196k cash prize" or adding a note.
Sources: `hackaday.io/prize/2014/rewards`, `hackaday.com/2014/11/13/satnogs-wins-the-2014-hackaday-prize/`.

## 4. ESA GENSO (ch02 L120)

Claim: "ESA GENSO project (started 2006, cancelled by 2014)"

**MOSTLY VERIFIED — start year slightly off.** GENSO kickoff was November 2007 at the CubeSat workshop (per ESA education pages + Wikipedia). The project was described by NASA as "on hold" by 2014 and is now cancelled. Bib entry `esa-genso` already uses "2007–2012" which is closer to correct. ch02 text says "started 2006" — **should be 2007**. "Cancelled by 2014" is defensible (on-hold = effectively cancelled).
Recommended fix: change "started 2006" → "started 2007" in ch02 L120.
Sources: `en.wikipedia.org/wiki/Global_Educational_Network_for_Satellite_Operations`, `esa.int/Education/Background_on_GENSO`.

## 5. Japanese Ground Station Network at U. Tokyo (ch02 L120)

Claim: "Japanese Ground Station Network at the University of Tokyo"

**VERIFIED.** University of Tokyo ISSL (Intelligent Space Systems Laboratory, Nakasuka) led a multi-university "Ground Station Network" (GSN) — distributed ground stations connected over internet for remote operation. `nakasuka2005tokyo` bib entry is the correct reference type (IAC paper).
Sources: `sciencedirect.com/science/article/abs/pii/S0094576507001786` (XI-V GSN paper), `jstage.jst.go.jp/article/tstj/7/ists26/7_ists26_Tu_31`.

## 6. SatNOGS component URLs (ch02 L103–109)

| URL in text | Status |
|---|---|
| `network.satnogs.org` | RESOLVES — station list page live |
| `db.satnogs.org` | RESOLVES — DB home page live |
| SatNOGS Client (no URL in description item) | N/A |
| SatNOGS Rotator (no URL in description item, cited via `satnogs-rotator-fw`) | N/A |

Bib entries `satnogs-main` (`satnogs.org/`), `satnogs-network`, `satnogs-wiki` (`wiki.satnogs.org/Main_Page`), `satnogs-rotator-fw` (`librespacefoundation.gitlab.io/...`) — Main wiki page confirmed live (last updated 2021-08-01).
**VERIFIED.**

## 7. rotctld TCP 4533 (ch02 L112)

Claim: "rotctld daemon exposes TCP interface on port 4533 (hamlib)"

**VERIFIED.** Default listening port for rotctld is 4533 per hamlib man page.
Source: `hamlib.sourceforge.net/html/rotctld.1.html`, `manpages.ubuntu.com/manpages/xenial/man8/rotctld.8.html`.

## 8. hamlib model 204 = EasyComm III (ch02 L112, L116; ch01 L38)

Claim: "hamlib model 204 = EasyComm III"

**VERIFIED.** `ROT_MODEL_EASYCOMM3` = ROT_MAKE_MODEL(ROT_EASYCOMM=2, 4) = 100·2 + 4 = **204**. Status: Stable. Used by SatNOGS rotator firmware.
Source: `hamlib.sourceforge.net/manuals/4.7/group__rotlist.html`, `github.com/Hamlib/Hamlib/wiki/Supported-Rotators`.

## 9. AetherSpace specs (ch01 L31, ch02 L42)

Claim: "3U CubeSat, KU Leuven, heat-shield sample-return, launch ~2030, 433 MHz UHF, +14 dBm (~25 mW), ~2 dBi turnstile/dipole, CC1200"

**PARTIALLY VERIFIED.**
- **3U CubeSat**: VERIFIED. Multiple sources confirm 3U form factor with inflatable fTPS heat shield.
- **KU Leuven**: VERIFIED. Student team from KU Leuven.
- **Heat-shield re-entry/sample return**: VERIFIED. "Re-entry CubeSat" is the project's primary description; sample-return is the stated long-term goal.
- **Launch ~2030**: NOT INDEPENDENTLY VERIFIED from public web (aetherspace.be landing page has a "REXUS Launch" countdown, but no dated CubeSat flight manifest). Thesis attributes this to "AetherSpace specification (2026-03-05)" — acceptable as private-communication citation but not publicly cross-checkable. Keep the citation anchor.
- **433 MHz, +14 dBm, 2 dBi turnstile/dipole, CC1200**: NOT PUBLICLY VERIFIABLE — all sourced from AetherSpace team private spec (2026-03-05). Public site (`aetherspace.be`) does not list RF parameters. This is a private-source claim; the thesis correctly attributes it.

Sources: `aetherspace.be/`, `aetherspace.be/public/plan`, `ieeexplore.ieee.org/document/11005409/` (heat-shield FBG paper), `researchgate.net/publication/391792587`.

## 10. Previous KU Leuven work (ch02 L42)

Claim: "Surkijn and Van Elst (2022), Hanssens (2023)"

**NOT INDEPENDENTLY VERIFIED** via public web. Both are KU Leuven masterproeven (internal archive). `hanssens2023` is in bib with full Dutch title. `surkijn2022` is in bib with plausible title. These are internal institutional references — trust level is on author, not cross-checkable online. Recommend confirming both entries exist in KU Leuven Limo repository before submission; no action needed unless citation errors surface.

## 11. Hanssens stepper torque finding (ch02 L154–156)

Claim: "Hanssens tested Stepperdrive MSD-50-5.6D stepper driver; insufficient torque for azimuth under wind"

**NOT INDEPENDENTLY VERIFIABLE** via public web (masterproef not online-indexed). Cited as `\cite{hanssens2023}`. Trust anchored to the actual thesis document at `Papers/`. Verify locally if needed.

## 12. Custom commands invisible to hamlib (ch05 L92)

Claim: "Custom commands are invisible to hamlib, so compatibility with rotctld, GPredict, and satnogs-client rotator path is maintained"

**VERIFIED (by design).** Hamlib EasyComm backend parses a fixed command set (AZ, EL, SA/SB, UP/DN, VE, VU, VL, VR, SS, MR, ML, RESET, etc. per EasyComm III). Unknown ASCII tokens received by the controller are simply ignored by hamlib (hamlib never sends them), so adding extensions at the firmware side is transparent. This is the standard SatNOGS rotator firmware approach.
Source: `github.com/Hamlib/Hamlib` easycomm.c backend; `librespacefoundation.gitlab.io/satnogs/satnogs-rotator-firmware/` protocol docs.

## 13. SatNOGS Docker ≥1 GB RAM (ch01 L45, ch02 L125)

Claim: "SatNOGS Docker stack targets ≥1 GB of RAM"

**NOT INDEPENDENTLY VERIFIED** at the exact "1 GB" figure. Public docs (`docs.satnogs.org/projects/satnogs-client/en/latest/docker.html`, `wiki.satnogs.org/Raspberry_Pi`) don't state a hard minimum RAM. Pi 3B/3B+/4/5 are all listed as reference platforms (Pi 3B+ has 1 GB; Pi 4 has 2–8 GB). The claim is a reasonable inference from "reference platforms ≥ 1 GB" but is not sourced from a single cited line. `satnogs-main` is cited after the claim in ch02 — that citation does not directly support "≥1 GB". Recommend either softening to "typically targets Pi 3B/4 reference images with ≥1 GB of RAM" or adding `\cite{satnogs-client-setup}` + `\cite{satnogs-hw-req}`.
Sources: `wiki.satnogs.org/Raspberry_Pi`, `docs.satnogs.org/projects/satnogs-client/en/latest/docker.html`.

## 14. `spreij2021aetherspace` bibliography entry (bib L411–418)

Claim: article by Spreij & Verlinden, 2021, journal "veto", about CubeSat re-entry.

**VERIFIED — with format issues.**
- Article exists: "The challenges of a CubeSat re-entry mission", published 2021-05-20 on `veto.be`.
- Authors: Roemer Spreij and Simon Verlinden — **VERIFIED**.
- "Veto" is a **student newspaper** (Belgian student magazine at KU Leuven), not a peer-reviewed journal. Bib entry types it as `@article{...journal={veto}}` which is technically misleading — this is closer to `@online` or `@misc` (web magazine article), not a refereed publication.
- URL in bib note is `https://aetherspace.be/` — should be the actual article: `https://www.veto.be/english/the-challenges-of-a-cubesat-re-entry-mission/105543`.

Recommended fix:
```bibtex
@online{spreij2021aetherspace,
  author = {Spreij, Roemer and Verlinden, Simon},
  title  = {The Challenges of a {CubeSat} Re-entry Mission},
  year   = {2021},
  month  = may,
  url    = {https://www.veto.be/english/the-challenges-of-a-cubesat-re-entry-mission/105543},
  note   = {Veto student magazine, KU Leuven}
}
```

## Summary of required fixes

| # | File / line | Fix |
|---|---|---|
| 4 | `ch02_background.tex` L120 | "started 2006" → "started 2007" |
| 3 | `bibliography.bib` L444–447 | $196,418 is the widely-cited figure; some sources say $198,418. Acceptable as-is; optional to soften. |
| 13 | `ch01_introduction.tex` L45 and `ch02_background.tex` L125 | Either soften "≥ 1 GB of RAM" claim or add a wiki/docs citation that supports it. `\cite{satnogs-main}` does not directly support it. |
| 14 | `bibliography.bib` L411–418 | Change `@article` → `@online`; fix URL to the actual Veto article; add authors' first names; note Veto is a student magazine not a journal. |
| 9 | `ch01` / `ch02` AetherSpace RF specs | Already correctly attributed to "AetherSpace specification (2026-03-05)" private comm. No change needed unless a public datasheet is now available. |

No load-bearing claims are wrong. All fixes are cosmetic/accuracy polish.
