# Project tuning debug

- mapa: `indoor_2`
- repeat: `9`
- start: `[3.200, 8.200, 1.642]`
- cil: `[7.000, 2.600]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1229`
- final goal error: `0.487 m`
- travelled: `21.910 m`
- runtime: `142.301 s`

## Maxima a minima

- max XY localization error: `8.522 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `4.162`
- max ambiguity counter: `54`
- ambiguity active steps: `440`
- min front lidar: `0.612 m`
- max track stall steps: `23`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `8.530 m`
- max contradiction counter: `20`
- max no-progress counter: `14`
- max trace-loop counter: `9`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `8 / 8`
- max reseed crisis score: `0.745`
- max disambiguation goal switches: `7`
- min path quality score: `0.863`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `0.507 m`
- final true goal distance: `0.503 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `84` kroku
- `globalize`: `91` kroku
- `path_entry`: `8` kroku
- `plan`: `2` kroku
- `relocalize`: `498` kroku
- `track`: `544` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `747`
- `pf_cluster_wide`: `356`
- `pf_cluster_very_wide`: `57`
- `scan_match_very_bad`: `34`
- `pf_cluster_medium`: `25`

## Nejcastejsi transition reasons

- `stay`: `1212`
- `disambiguate_clear_to_relocalize`: `3`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `relocalize_hard_reset_bad_hypothesis`: `2`
- `relocalize_to_disambiguate_ambiguity`: `2`
- `commit_to_verify`: `1`
- `disambiguate_hard_reset_bad_hypothesis`: `1`

## Nejcastejsi forced replans

- `none`: `1229`

## Nejcastejsi forced relocalize

- `none`: `1228`
- `false_goal_deadlock`: `1`
