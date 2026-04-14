# Project tuning debug

- mapa: `indoor_1`
- repeat: `2`
- start: `[2.000, 0.800, -0.457]`
- cil: `[9.200, 9.200]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `5.308 m`
- travelled: `35.857 m`
- runtime: `154.369 s`

## Maxima a minima

- max XY localization error: `0.897 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.521`
- max ambiguity counter: `83`
- ambiguity active steps: `641`
- min front lidar: `0.896 m`
- max track stall steps: `81`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `4`
- max estimate jump: `1.218 m`
- max contradiction counter: `20`
- max no-progress counter: `12`
- max trace-loop counter: `4`
- max commit watchdog counter: `0`
- max hard path distance counter: `8`
- reseed attempts / successes: `5 / 5`
- max reseed crisis score: `0.925`
- max disambiguation goal switches: `8`
- min path quality score: `0.789`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `5.231 m`
- final true goal distance: `5.290 m`

## Stavy

- `commit`: `2` kroku
- `disambiguate`: `145` kroku
- `globalize`: `146` kroku
- `path_entry`: `16` kroku
- `plan`: `4` kroku
- `relocalize`: `681` kroku
- `track`: `1054` kroku
- `verify`: `2` kroku

## Nejcastejsi reasons

- `ok`: `1520`
- `pf_cluster_wide`: `382`
- `pf_cluster_very_wide`: `123`
- `pf_cluster_medium`: `18`
- `scan_match_warn`: `3`

## Nejcastejsi transition reasons

- `stay`: `2017`
- `disambiguate_clear_to_relocalize`: `5`
- `path_entry_to_track_merged`: `4`
- `plan_to_path_entry_path_ready`: `4`
- `relocalize_hard_reset_bad_hypothesis`: `4`
- `relocalize_to_disambiguate_ambiguity`: `3`
- `commit_to_verify`: `2`
- `relocalize_finish_to_commit`: `2`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2046`
- `false_goal_deadlock`: `2`
- `hard_path_distance`: `1`
- `track_uncommitted`: `1`
