# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `9.138 m`
- travelled: `29.812 m`
- runtime: `129.801 s`

## Maxima a minima

- max XY localization error: `11.668 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `44.598`
- max ambiguity counter: `60`
- ambiguity active steps: `757`
- min front lidar: `0.775 m`
- max track stall steps: `115`
- max scan mismatch counter: `4`
- max map conflict counter: `0`
- max path revision: `5`
- max estimate jump: `6.093 m`
- max contradiction counter: `20`
- max no-progress counter: `11`
- max trace-loop counter: `5`
- max commit watchdog counter: `0`
- max hard path distance counter: `3`
- reseed attempts / successes: `9 / 9`
- max reseed crisis score: `0.100`
- max disambiguation goal switches: `3`
- min path quality score: `0.605`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `9.115 m`
- final true goal distance: `9.131 m`

## Stavy

- `commit`: `2` kroku
- `disambiguate`: `79` kroku
- `globalize`: `79` kroku
- `path_entry`: `32` kroku
- `plan`: `5` kroku
- `relocalize`: `1077` kroku
- `track`: `774` kroku
- `verify`: `2` kroku

## Nejcastejsi reasons

- `ok`: `1457`
- `pf_cluster_wide`: `280`
- `pf_cluster_very_wide`: `232`
- `scan_match_very_bad`: `34`
- `pf_cluster_medium`: `17`

## Nejcastejsi transition reasons

- `stay`: `2018`
- `path_entry_to_track_merged`: `5`
- `plan_to_path_entry_path_ready`: `5`
- `relocalize_hard_reset_bad_hypothesis`: `5`
- `disambiguate_clear_to_relocalize`: `3`
- `commit_to_verify`: `2`
- `relocalize_finish_to_commit`: `2`
- `relocalize_to_disambiguate_ambiguity`: `2`

## Nejcastejsi forced replans

- `none`: `2049`
- `away_from_goal`: `1`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `false_goal_deadlock`: `1`
