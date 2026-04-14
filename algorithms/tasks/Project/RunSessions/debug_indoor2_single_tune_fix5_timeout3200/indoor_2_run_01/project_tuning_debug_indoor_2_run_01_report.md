# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `3200` kroku
- vysledek: `timeout`
- kroky: `3200`
- final goal error: `8.529 m`
- travelled: `38.912 m`
- runtime: `351.197 s`

## Maxima a minima

- max XY localization error: `11.360 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `31.727`
- max ambiguity counter: `62`
- ambiguity active steps: `757`
- min front lidar: `0.399 m`
- max track stall steps: `181`
- max scan mismatch counter: `6`
- max map conflict counter: `0`
- max path revision: `7`
- max estimate jump: `6.061 m`
- max contradiction counter: `20`
- max no-progress counter: `18`
- max trace-loop counter: `3`
- max commit watchdog counter: `1`
- max hard path distance counter: `8`
- reseed attempts / successes: `11 / 11`
- max reseed crisis score: `0.110`
- max disambiguation goal switches: `19`
- min path quality score: `0.705`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `5.015 m`
- final true goal distance: `8.520 m`

## Stavy

- `commit`: `4` kroku
- `disambiguate`: `228` kroku
- `globalize`: `189` kroku
- `path_entry`: `28` kroku
- `plan`: `7` kroku
- `relocalize`: `1489` kroku
- `track`: `1251` kroku
- `verify`: `4` kroku

## Nejcastejsi reasons

- `ok`: `2374`
- `pf_cluster_wide`: `409`
- `pf_cluster_very_wide`: `265`
- `pf_cluster_medium`: `78`
- `scan_match_very_bad`: `41`

## Nejcastejsi transition reasons

- `stay`: `3144`
- `relocalize_hard_reset_bad_hypothesis`: `8`
- `disambiguate_clear_to_relocalize`: `7`
- `path_entry_to_track_merged`: `7`
- `plan_to_path_entry_path_ready`: `7`
- `relocalize_to_disambiguate_ambiguity`: `5`
- `commit_to_verify`: `4`
- `relocalize_finish_to_commit`: `4`

## Nejcastejsi forced replans

- `none`: `3198`
- `track_stall`: `2`

## Nejcastejsi forced relocalize

- `none`: `3198`
- `false_goal_uncommitted`: `1`
- `hard_path_distance`: `1`
