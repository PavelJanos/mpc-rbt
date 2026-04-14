# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `7.224 m`
- travelled: `25.878 m`
- runtime: `229.221 s`

## Maxima a minima

- max XY localization error: `12.207 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `165.262`
- max ambiguity counter: `30`
- ambiguity active steps: `733`
- min front lidar: `0.529 m`
- max track stall steps: `95`
- max scan mismatch counter: `12`
- max map conflict counter: `0`
- max path revision: `5`
- max estimate jump: `6.061 m`
- max contradiction counter: `20`
- max no-progress counter: `6`
- max trace-loop counter: `3`
- max commit watchdog counter: `0`
- max hard path distance counter: `2`
- reseed attempts / successes: `8 / 8`
- max reseed crisis score: `0.096`
- max disambiguation goal switches: `4`
- min path quality score: `0.621`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `7.236 m`
- final true goal distance: `7.228 m`

## Stavy

- `commit`: `3` kroku
- `disambiguate`: `96` kroku
- `globalize`: `110` kroku
- `path_entry`: `20` kroku
- `plan`: `5` kroku
- `relocalize`: `1137` kroku
- `track`: `676` kroku
- `verify`: `3` kroku

## Nejcastejsi reasons

- `ok`: `1381`
- `pf_cluster_wide`: `411`
- `pf_cluster_very_wide`: `120`
- `scan_match_very_bad`: `46`
- `pf_cluster_medium`: `36`

## Nejcastejsi transition reasons

- `stay`: `2012`
- `relocalize_hard_reset_bad_hypothesis`: `6`
- `path_entry_to_track_merged`: `5`
- `plan_to_path_entry_path_ready`: `5`
- `disambiguate_clear_to_relocalize`: `4`
- `commit_to_verify`: `3`
- `relocalize_finish_to_commit`: `3`
- `verify_to_plan`: `3`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `false_goal_deadlock`: `1`
