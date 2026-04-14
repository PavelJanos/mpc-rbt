# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `3200` kroku
- vysledek: `timeout`
- kroky: `3200`
- final goal error: `1.750 m`
- travelled: `45.028 m`
- runtime: `599.113 s`

## Maxima a minima

- max XY localization error: `9.216 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `10.670`
- max ambiguity counter: `66`
- ambiguity active steps: `1116`
- min front lidar: `0.239 m`
- max track stall steps: `181`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `11`
- max estimate jump: `7.736 m`
- max contradiction counter: `20`
- max no-progress counter: `22`
- max trace-loop counter: `14`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `18 / 18`
- max reseed crisis score: `0.730`
- max disambiguation goal switches: `20`
- min path quality score: `0.705`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `1.789 m`
- final true goal distance: `1.758 m`

## Stavy

- `commit`: `6` kroku
- `disambiguate`: `311` kroku
- `globalize`: `314` kroku
- `path_entry`: `44` kroku
- `plan`: `11` kroku
- `relocalize`: `1628` kroku
- `track`: `880` kroku
- `verify`: `6` kroku

## Nejcastejsi reasons

- `ok`: `1808`
- `pf_cluster_wide`: `746`
- `pf_cluster_very_wide`: `378`
- `pf_cluster_medium`: `205`
- `scan_match_very_bad`: `25`

## Nejcastejsi transition reasons

- `stay`: `3116`
- `disambiguate_clear_to_relocalize`: `11`
- `path_entry_to_track_merged`: `11`
- `plan_to_path_entry_path_ready`: `11`
- `relocalize_hard_reset_bad_hypothesis`: `10`
- `localize_to_disambiguate_ambiguity`: `7`
- `commit_to_verify`: `6`
- `relocalize_finish_to_commit`: `6`

## Nejcastejsi forced replans

- `none`: `3200`

## Nejcastejsi forced relocalize

- `none`: `3191`
- `track_uncommitted`: `6`
- `false_goal_deadlock`: `1`
- `false_goal_offpath`: `1`
- `track_stall_false_goal`: `1`
