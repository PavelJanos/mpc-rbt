# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `wall`
- kroky: `1931`
- final goal error: `7.483 m`
- travelled: `18.811 m`
- runtime: `269.906 s`

## Maxima a minima

- max XY localization error: `11.227 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `9.856`
- max ambiguity counter: `37`
- ambiguity active steps: `577`
- min front lidar: `0.347 m`
- max track stall steps: `181`
- max scan mismatch counter: `2`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `6.061 m`
- max contradiction counter: `20`
- max no-progress counter: `10`
- max trace-loop counter: `9`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `10 / 10`
- max reseed crisis score: `0.781`
- max disambiguation goal switches: `13`
- min path quality score: `1.000`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `6.621 m`
- final true goal distance: `7.466 m`

## Stavy

- `commit`: `2` kroku
- `disambiguate`: `181` kroku
- `globalize`: `309` kroku
- `path_entry`: `8` kroku
- `plan`: `2` kroku
- `relocalize`: `1173` kroku
- `track`: `255` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1172`
- `pf_cluster_wide`: `421`
- `pf_cluster_very_wide`: `213`
- `pf_cluster_medium`: `59`
- `scan_match_very_bad`: `33`

## Nejcastejsi transition reasons

- `stay`: `1899`
- `disambiguate_clear_to_relocalize`: `7`
- `relocalize_hard_reset_bad_hypothesis`: `6`
- `relocalize_to_disambiguate_ambiguity`: `4`
- `globalize_to_disambiguate_probe`: `2`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `relocalize_finish_to_commit`: `2`

## Nejcastejsi forced replans

- `none`: `1931`

## Nejcastejsi forced relocalize

- `none`: `1929`
- `false_goal_deadlock`: `1`
- `track_stall_false_goal`: `1`
