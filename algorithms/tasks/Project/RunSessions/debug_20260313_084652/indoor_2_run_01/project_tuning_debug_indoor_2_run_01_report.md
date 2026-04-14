# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `6.363 m`
- travelled: `28.158 m`
- runtime: `151.059 s`

## Maxima a minima

- max XY localization error: `12.066 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.042`
- max ambiguity counter: `120`
- ambiguity active steps: `952`
- min front lidar: `0.638 m`
- max track stall steps: `18`
- max scan mismatch counter: `2`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `9.106 m`
- max contradiction counter: `20`
- max no-progress counter: `5`
- max trace-loop counter: `4`
- max commit watchdog counter: `3`
- max hard path distance counter: `0`
- reseed attempts / successes: `14 / 14`
- max reseed crisis score: `0.092`
- max disambiguation goal switches: `5`
- min path quality score: `0.701`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `6.377 m`
- final true goal distance: `6.344 m`

## Stavy

- `commit`: `2` kroku
- `disambiguate`: `262` kroku
- `globalize`: `81` kroku
- `path_entry`: `8` kroku
- `plan`: `2` kroku
- `relocalize`: `1344` kroku
- `track`: `351` kroku

## Nejcastejsi reasons

- `ok`: `1092`
- `pf_cluster_wide`: `598`
- `pf_cluster_very_wide`: `253`
- `pf_cluster_medium`: `60`
- `scan_match_very_bad`: `24`

## Nejcastejsi transition reasons

- `stay`: `2020`
- `relocalize_hard_reset_bad_hypothesis`: `11`
- `disambiguate_clear_to_relocalize`: `4`
- `relocalize_to_disambiguate_ambiguity`: `3`
- `commit_to_plan`: `2`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `relocalize_finish_to_commit`: `2`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `false_goal_deadlock`: `1`
