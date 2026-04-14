# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `wall`
- kroky: `915`
- final goal error: `7.438 m`
- travelled: `14.030 m`
- runtime: `156.793 s`

## Maxima a minima

- max XY localization error: `9.319 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `6.862`
- max ambiguity counter: `22`
- ambiguity active steps: `187`
- min front lidar: `0.111 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `7.809 m`
- max contradiction counter: `20`
- max no-progress counter: `6`
- max trace-loop counter: `0`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.033 m`
- final estimated goal distance: `7.339 m`
- final true goal distance: `7.420 m`

## Stavy

- `confirm`: `70` kroku
- `disambiguate`: `72` kroku
- `globalize`: `160` kroku
- `plan`: `1` kroku
- `relocalize`: `611` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `417`
- `pf_cluster_wide`: `213`
- `pf_cluster_very_wide`: `177`
- `pf_cluster_medium`: `67`
- `scan_match_very_bad`: `18`

## Nejcastejsi transition reasons

- `stay`: `899`
- `disambiguate_clear_to_relocalize`: `3`
- `localize_hard_reset_bad_hypothesis`: `2`
- `localize_to_disambiguate_ambiguity`: `2`
- `relocalize_gate_to_confirm`: `2`
- `relocalize_hard_reset_bad_hypothesis`: `2`
- `confirm_to_globalize_drop`: `1`
- `confirm_to_verify`: `1`

## Nejcastejsi forced replans

- `none`: `915`

## Nejcastejsi forced relocalize

- `none`: `915`
