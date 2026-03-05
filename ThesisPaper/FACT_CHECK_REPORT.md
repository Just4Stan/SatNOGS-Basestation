# Merged Verification of `ThesisPaper/FACT_CHECK_REPORT.md`

Date: 2026-03-05
Scope: concise re-check of the original report against the current repo and live official sources.

## Bottom Line

The original `FACT_CHECK_REPORT.md` is directionally sound, but it is too confident in a few places.

Main corrections:
- Treat the method section as `UNVERIFIED`: the cited evidence sources exist, but the report does not show an auditable evidence trail.
- Soften `largely validated` for the rotator stack to `implemented, with self-reported validation results`, because raw logs and measurement artifacts are limited in-repo.
- Keep RF HAT and full-station claims clearly separated into `design-complete` vs `hardware-validated`.
- Keep battery runtime labeled as an analytical projection, not a measured field result.
- Do not call MDN a primary source. It is a good secondary source; the W3C Geolocation Recommendation is the primary standard.

## Current Web Recheck

Access date for all items below: 2026-03-05.

| Topic | Current live status | Source |
|---|---|---|
| Hamlib model 204 | Hamlib's current supported-rotators list still shows `204  Hamlib  EasycommIII  Stable`. | https://github.com/Hamlib/Hamlib/wiki/Supported-Rotators |
| SatNOGS rotator setup | SatNOGS setup docs still reference Hamlib rotator configuration and explicitly show `-m 204 ...` for a SatNOGS rotator. | https://wiki.satnogs.org/SatNOGS_Setup |
| SatNOGS network size | The live ground-stations page currently paginates to page `206` and returns `20` stations on the current page. The exact total is dynamic and should be date-stamped if quoted. | https://network.satnogs.org/stations/ |
| TI CC1200 status | TI still lists `CC1200` as `ACTIVE`. The product page currently lists best sensitivity `-123 dBm` and max TX power `+16 dBm`. | https://www.ti.com/product/CC1200 |
| Geolocation secure context | MDN still states that the Geolocation API is available only in secure contexts. The W3C Geolocation spec is currently a Recommendation. | https://developer.mozilla.org/en-US/docs/Web/API/Geolocation_API ; https://www.w3.org/TR/geolocation/ |
| AetherSpace public status | AetherSpace's public site still describes a Belgian student CubeSat team focused on re-entry. The public site does not verify detailed values such as `3U`, `2030`, or `14 dBm`. | https://aetherspace.be/ ; https://aetherspace.be/public/plan |

## Local Recheck

| Item | Current repo status | Local evidence |
|---|---|---|
| `station.service` mode | Still runs `station.py --daemon --no-rf --doppler`. | `Pi/services/station.service:10` |
| RF HAT hardware status | Still pending assembly / bring-up / VHF tuning. | `PROGRESS.md:49-51`; `ThesisPaper/draft.md:2479-2491` |
| SatNOGS commissioning | Still pending registration / client configuration. | `PROGRESS.md:183-187` |
| Battery runtime wording | Still presented as an estimate/projection in the thesis draft. | `ThesisPaper/draft.md:2458-2477`; `ThesisPaper/draft.md:2517` |
| Cost wording | RF HAT and complete-station totals are still explicitly marked as estimates. | `ThesisPaper/draft.md:2309-2384` |
| Python LOC drift | Current counts are `station.py 1104`, `dashboard.py 882`, `rf_hat.py 825`, total Python stack `3988`. | local `wc -l` recomputation |
| `PROGRESS.md` reliability | Useful, but stale in at least one place: it still says thesis prose is not yet written while `draft.md` is populated. | `PROGRESS.md:246-248`; `ThesisPaper/draft.md:1-2600+` |

## Verdict on the Original Report

| Original report claim | Verdict | Why |
|---|---|---|
| It checked the listed local and external sources | `UNVERIFIED` | The source list is plausible, but the report itself does not show an evidence log. |
| No edits were made to `draft.md` | `UNVERIFIED` | No auditable proof in-repo. |
| Rotator mechanics + PCB + firmware are `largely validated` | `PARTLY TRUE` | Implemented and self-reported as tested, but raw artifacts are limited. |
| EasyComm / hamlib compatibility is validated at subsystem level | `TRUE` | Firmware, `rotctld` config, Hamlib model 204, and SatNOGS docs align. |
| Pi control/dashboard stack is implemented and coherent | `TRUE` | Code, services, and draft architecture are consistent. |
| RF HAT firmware/protocol design is implemented | `PARTLY TRUE` | Protocol logic is present; independent firmware build reproduction was blocked by missing network access for PlatformIO dependencies. |
| RF HAT hardware RF performance is not yet validated | `TRUE` | Supported by repo status and draft wording. |
| Full SatNOGS commissioning is not yet validated | `TRUE` | Registration, client configuration, and RF-path integration remain pending. |
| Battery runtime is estimated, not fully measured | `TRUE` | The thesis still presents it as a formula-based estimate. |
| MDN Geolocation API is a primary source | `PARTLY TRUE` | Useful source, but secondary. W3C is primary. |

## Recommended Replacement for `FACT_CHECK_REPORT.md`

Use the text below as the concise replacement.

---

# Fact-Check Report for `ThesisPaper/draft.md`

Date: 2026-03-05
Scope: factual correctness, internal consistency, implementation alignment, and claim defensibility.

## Summary

- The report is broadly correct on subsystem maturity.
- The main risk is overconfidence, not outright falsehood.
- The rotator stack is implemented and supported by self-reported validation results, but raw validation artifacts are limited.
- The RF HAT is design-complete in schematic/layout/BOM/firmware terms, but RF hardware validation is still pending.
- Full SatNOGS commissioning is still pending.
- Battery runtime and full-station cost should remain explicitly labeled as estimates.

## Current Verified Status

| Subsystem | Status | Confidence | Evidence |
|---|---|---|---|
| Rotator mechanics + motor PCB + firmware | Implemented; self-reported validation results present | Medium-High | `PROGRESS.md`, firmware sources, thesis Chapter 10 |
| EasyComm / hamlib compatibility | Validated at subsystem level | High | firmware command set, `rotctld` service usage, Hamlib model 204 |
| Pi control/dashboard stack | Implemented and coherent | High | `Pi/` code, services, local syntax check |
| RF HAT firmware/protocol | Implemented | High | protocol sources and docs present |
| RF HAT hardware RF performance | Not yet validated on hardware | High | bring-up and VHF tuning still pending |
| Full SatNOGS network commissioning | Not yet validated | High | registration and client setup still pending |
| Battery runtime | Projection, not measured field runtime | High | formula-based estimate in thesis |

## Required Wording Fixes

1. Replace `largely validated` with `implemented, with self-reported validation results` unless raw logs/plots are added.
2. Keep RF claims explicitly labeled as `measured`, `reference-derived`, or `estimated`.
3. Keep full-station cost totals labeled as estimates.
4. Keep battery runtime labeled as a projection.
5. Date-stamp any live SatNOGS network counts.
6. Treat MDN as a secondary source; use the W3C Geolocation Recommendation as the primary standard.

## Current External Spot Checks

Accessed 2026-03-05:
- Hamlib supported rotators: model `204` is still `EasycommIII`.
- SatNOGS setup docs still show `-m 204` for a SatNOGS rotator.
- SatNOGS ground-stations list is live and dynamic; it currently paginates to page `206`.
- TI still lists `CC1200` as `ACTIVE`.
- MDN still states Geolocation requires a secure context; W3C Geolocation is a current Recommendation.
- AetherSpace's public site confirms the re-entry CubeSat project, but not detailed RF/link-budget assumptions.

## Notes

- `PROGRESS.md` should not be treated as a sole authority; it is partly stale.
- Recompute LOC counts before submission.
- If stronger validation language is desired, add raw measurement artifacts and a claim-to-evidence appendix.
