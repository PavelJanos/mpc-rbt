# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `1600` kroku
- vysledek: `wall`
- kroky: `866`
- final goal error: `7.513 m`
- travelled: `10.792 m`
- runtime: `70.983 s`

## Maxima a minima

- max XY localization error: `9.267 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `5.062`
- max ambiguity counter: `20`
- ambiguity active steps: `196`
- min front lidar: `0.725 m`
- max track stall steps: `69`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `7.677 m`
- max contradiction counter: `17`
- max no-progress counter: `14`
- max trace-loop counter: `7`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `3 / 3`
- max reseed crisis score: `0.062`
- max disambiguation goal switches: `1`
- min path quality score: `1.000`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `7.431 m`
- final true goal distance: `7.501 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `51` kroku
- `globalize`: `79` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `631` kroku
- `track`: `98` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `588`
- `pf_cluster_wide`: `136`
- `pf_cluster_very_wide`: `101`
- `pf_not_unique`: `13`
- `pf_cluster_medium`: `11`

## Nejcastejsi transition reasons

- `stay`: `853`
- `disambiguate_clear_to_relocalize`: `2`
- `relocalize_hard_reset_bad_hypothesis`: `2`
- `commit_to_verify`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `866`

## Nejcastejsi forced relocalize

- `none`: `865`
- `false_goal_deadlock`: `1`
