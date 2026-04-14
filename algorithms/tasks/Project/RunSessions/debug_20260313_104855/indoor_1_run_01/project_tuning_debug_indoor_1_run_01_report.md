# Project tuning debug

- mapa: `indoor_1`
- repeat: `1`
- start: `[9.200, 8.400, 0.265]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `7.961 m`
- travelled: `18.627 m`
- runtime: `120.551 s`

## Maxima a minima

- max XY localization error: `11.060 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.345`
- max ambiguity counter: `156`
- ambiguity active steps: `552`
- min front lidar: `0.707 m`
- max track stall steps: `181`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `6`
- max estimate jump: `10.754 m`
- max contradiction counter: `20`
- max no-progress counter: `16`
- max trace-loop counter: `3`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `7 / 7`
- max reseed crisis score: `0.533`
- max disambiguation goal switches: `12`
- min path quality score: `1.000`
- min trace bbox diag: `0.034 m`
- final estimated goal distance: `7.943 m`
- final true goal distance: `7.960 m`

## Stavy

- `commit`: `5` kroku
- `disambiguate`: `150` kroku
- `globalize`: `353` kroku
- `path_entry`: `24` kroku
- `plan`: `6` kroku
- `relocalize`: `384` kroku
- `track`: `1123` kroku
- `verify`: `5` kroku

## Nejcastejsi reasons

- `ok`: `1544`
- `pf_cluster_wide`: `283`
- `pf_cluster_very_wide`: `186`
- `pf_cluster_medium`: `15`
- `scan_match_warn`: `11`

## Nejcastejsi transition reasons

- `stay`: `2004`
- `path_entry_to_track_merged`: `6`
- `plan_to_path_entry_path_ready`: `6`
- `commit_to_verify`: `5`
- `disambiguate_clear_to_relocalize`: `5`
- `relocalize_finish_to_commit`: `5`
- `verify_to_plan`: `5`
- `relocalize_hard_reset_bad_hypothesis`: `4`

## Nejcastejsi forced replans

- `none`: `2049`
- `track_stall`: `1`

## Nejcastejsi forced relocalize

- `none`: `2048`
- `false_goal_deadlock`: `1`
- `track_stall_false_goal`: `1`
