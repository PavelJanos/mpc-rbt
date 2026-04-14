# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `5.675 m`
- travelled: `19.940 m`
- runtime: `213.616 s`

## Maxima a minima

- max XY localization error: `8.865 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `5.417`
- max ambiguity counter: `23`
- ambiguity active steps: `487`
- min front lidar: `0.434 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `6.061 m`
- max contradiction counter: `20`
- max no-progress counter: `19`
- max trace-loop counter: `1`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `5.683 m`
- final true goal distance: `5.675 m`

## Stavy

- `confirm`: `772` kroku
- `disambiguate`: `144` kroku
- `globalize`: `328` kroku
- `relocalize`: `806` kroku

## Nejcastejsi reasons

- `ok`: `1293`
- `pf_cluster_wide`: `458`
- `pf_cluster_very_wide`: `177`
- `pf_cluster_medium`: `61`
- `scan_match_very_bad`: `48`

## Nejcastejsi transition reasons

- `stay`: `2021`
- `disambiguate_clear_to_relocalize`: `6`
- `relocalize_hard_reset_bad_hypothesis`: `6`
- `localize_hard_reset_bad_hypothesis`: `4`
- `relocalize_gate_to_confirm`: `4`
- `confirm_to_globalize_drop`: `3`
- `localize_to_disambiguate_ambiguity`: `3`
- `relocalize_to_disambiguate_ambiguity`: `2`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
