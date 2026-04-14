# Project tuning debug

- mapa: `indoor_1`
- repeat: `1`
- start: `[9.200, 8.400, 0.265]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `5.813 m`
- travelled: `17.226 m`
- runtime: `168.501 s`

## Maxima a minima

- max XY localization error: `11.060 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `6.158`
- max ambiguity counter: `156`
- ambiguity active steps: `532`
- min front lidar: `0.707 m`
- max track stall steps: `181`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `7`
- max estimate jump: `10.754 m`
- max contradiction counter: `20`
- max no-progress counter: `22`
- max trace-loop counter: `17`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `7 / 7`
- max reseed crisis score: `0.533`
- max disambiguation goal switches: `14`
- min path quality score: `1.000`
- min trace bbox diag: `0.035 m`
- final estimated goal distance: `5.811 m`
- final true goal distance: `5.813 m`

## Stavy

- `commit`: `5` kroku
- `disambiguate`: `164` kroku
- `globalize`: `314` kroku
- `path_entry`: `26` kroku
- `plan`: `7` kroku
- `relocalize`: `321` kroku
- `track`: `1208` kroku
- `verify`: `5` kroku

## Nejcastejsi reasons

- `ok`: `1580`
- `pf_cluster_wide`: `269`
- `pf_cluster_very_wide`: `190`
- `pf_cluster_medium`: `5`
- `pf_weakly_unique`: `2`

## Nejcastejsi transition reasons

- `stay`: `2002`
- `plan_to_path_entry_path_ready`: `7`
- `path_entry_to_track_merged`: `6`
- `commit_to_verify`: `5`
- `disambiguate_clear_to_relocalize`: `5`
- `relocalize_finish_to_commit`: `5`
- `verify_to_plan`: `5`
- `relocalize_hard_reset_bad_hypothesis`: `3`

## Nejcastejsi forced replans

- `none`: `2048`
- `track_stall`: `2`

## Nejcastejsi forced relocalize

- `none`: `2048`
- `false_goal_deadlock`: `1`
- `track_stall_false_goal`: `1`
