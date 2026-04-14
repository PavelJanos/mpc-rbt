# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `6.097 m`
- travelled: `26.015 m`
- runtime: `105.502 s`

## Maxima a minima

- max XY localization error: `12.066 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.042`
- max ambiguity counter: `120`
- ambiguity active steps: `932`
- min front lidar: `0.638 m`
- max track stall steps: `18`
- max scan mismatch counter: `2`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `9.679 m`
- max contradiction counter: `20`
- max no-progress counter: `5`
- max trace-loop counter: `4`
- max commit watchdog counter: `3`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `0.092`
- max disambiguation goal switches: `1`
- min path quality score: `0.701`
- min trace bbox diag: `0.033 m`
- final estimated goal distance: `6.111 m`
- final true goal distance: `6.098 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `261` kroku
- `globalize`: `81` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `1418` kroku
- `track`: `284` kroku

## Nejcastejsi reasons

- `ok`: `1025`
- `pf_cluster_wide`: `592`
- `pf_cluster_very_wide`: `334`
- `pf_cluster_medium`: `45`
- `scan_match_very_bad`: `23`

## Nejcastejsi transition reasons

- `stay`: `2024`
- `relocalize_hard_reset_bad_hypothesis`: `11`
- `disambiguate_clear_to_relocalize`: `4`
- `relocalize_to_disambiguate_ambiguity`: `3`
- `commit_to_plan`: `1`
- `disambiguate_hard_reset_bad_hypothesis`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `false_goal_deadlock`: `1`
