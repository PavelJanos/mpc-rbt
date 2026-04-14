# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `11.381 m`
- travelled: `25.465 m`
- runtime: `166.701 s`

## Maxima a minima

- max XY localization error: `10.872 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `31.727`
- max ambiguity counter: `35`
- ambiguity active steps: `445`
- min front lidar: `0.399 m`
- max track stall steps: `181`
- max scan mismatch counter: `2`
- max map conflict counter: `0`
- max path revision: `5`
- max estimate jump: `6.061 m`
- max contradiction counter: `20`
- max no-progress counter: `5`
- max trace-loop counter: `3`
- max commit watchdog counter: `1`
- max hard path distance counter: `8`
- reseed attempts / successes: `6 / 6`
- max reseed crisis score: `0.097`
- max disambiguation goal switches: `8`
- min path quality score: `0.705`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `0.553 m`
- final true goal distance: `11.381 m`

## Stavy

- `commit`: `3` kroku
- `disambiguate`: `96` kroku
- `globalize`: `82` kroku
- `path_entry`: `20` kroku
- `plan`: `5` kroku
- `relocalize`: `919` kroku
- `track`: `922` kroku
- `verify`: `3` kroku

## Nejcastejsi reasons

- `ok`: `1583`
- `pf_cluster_wide`: `214`
- `pf_cluster_very_wide`: `142`
- `pf_cluster_medium`: `70`
- `scan_match_very_bad`: `19`

## Nejcastejsi transition reasons

- `stay`: `2012`
- `path_entry_to_track_merged`: `5`
- `plan_to_path_entry_path_ready`: `5`
- `relocalize_hard_reset_bad_hypothesis`: `5`
- `disambiguate_clear_to_relocalize`: `4`
- `commit_to_verify`: `3`
- `relocalize_finish_to_commit`: `3`
- `relocalize_to_disambiguate_ambiguity`: `3`

## Nejcastejsi forced replans

- `none`: `2049`
- `track_stall`: `1`

## Nejcastejsi forced relocalize

- `none`: `2048`
- `false_goal_uncommitted`: `1`
- `hard_path_distance`: `1`
