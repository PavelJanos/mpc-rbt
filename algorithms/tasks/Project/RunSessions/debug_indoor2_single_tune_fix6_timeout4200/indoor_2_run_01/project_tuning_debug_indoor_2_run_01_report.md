# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `4200` kroku
- vysledek: `timeout`
- kroky: `4200`
- final goal error: `7.639 m`
- travelled: `56.461 m`
- runtime: `723.964 s`

## Maxima a minima

- max XY localization error: `9.216 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `10.670`
- max ambiguity counter: `66`
- ambiguity active steps: `1466`
- min front lidar: `0.239 m`
- max track stall steps: `181`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `12`
- max estimate jump: `8.158 m`
- max contradiction counter: `20`
- max no-progress counter: `22`
- max trace-loop counter: `14`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `21 / 21`
- max reseed crisis score: `0.730`
- max disambiguation goal switches: `26`
- min path quality score: `0.705`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `7.608 m`
- final true goal distance: `7.641 m`

## Stavy

- `commit`: `7` kroku
- `disambiguate`: `461` kroku
- `globalize`: `434` kroku
- `path_entry`: `48` kroku
- `plan`: `12` kroku
- `relocalize`: `2334` kroku
- `track`: `898` kroku
- `verify`: `6` kroku

## Nejcastejsi reasons

- `ok`: `2388`
- `pf_cluster_wide`: `1008`
- `pf_cluster_very_wide`: `516`
- `pf_cluster_medium`: `210`
- `scan_match_very_bad`: `28`

## Nejcastejsi transition reasons

- `stay`: `4098`
- `disambiguate_clear_to_relocalize`: `15`
- `relocalize_hard_reset_bad_hypothesis`: `14`
- `path_entry_to_track_merged`: `12`
- `plan_to_path_entry_path_ready`: `12`
- `localize_to_disambiguate_ambiguity`: `7`
- `relocalize_finish_to_commit`: `7`
- `relocalize_to_disambiguate_ambiguity`: `7`

## Nejcastejsi forced replans

- `none`: `4200`

## Nejcastejsi forced relocalize

- `none`: `4190`
- `track_uncommitted`: `6`
- `false_goal_deadlock`: `2`
- `false_goal_offpath`: `1`
- `track_stall_false_goal`: `1`
