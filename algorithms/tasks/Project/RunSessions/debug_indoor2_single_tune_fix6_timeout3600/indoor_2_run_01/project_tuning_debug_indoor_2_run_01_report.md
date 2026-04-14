# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `3600` kroku
- vysledek: `timeout`
- kroky: `3600`
- final goal error: `1.079 m`
- travelled: `48.411 m`
- runtime: `492.074 s`

## Maxima a minima

- max XY localization error: `9.216 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `10.670`
- max ambiguity counter: `66`
- ambiguity active steps: `1186`
- min front lidar: `0.239 m`
- max track stall steps: `181`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `12`
- max estimate jump: `7.736 m`
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
- final estimated goal distance: `1.088 m`
- final true goal distance: `1.079 m`

## Stavy

- `commit`: `7` kroku
- `disambiguate`: `335` kroku
- `globalize`: `434` kroku
- `path_entry`: `48` kroku
- `plan`: `12` kroku
- `relocalize`: `1860` kroku
- `track`: `898` kroku
- `verify`: `6` kroku

## Nejcastejsi reasons

- `ok`: `2112`
- `pf_cluster_wide`: `804`
- `pf_cluster_very_wide`: `413`
- `pf_cluster_medium`: `205`
- `scan_match_very_bad`: `25`

## Nejcastejsi transition reasons

- `stay`: `3508`
- `disambiguate_clear_to_relocalize`: `12`
- `path_entry_to_track_merged`: `12`
- `plan_to_path_entry_path_ready`: `12`
- `relocalize_hard_reset_bad_hypothesis`: `11`
- `localize_to_disambiguate_ambiguity`: `7`
- `relocalize_finish_to_commit`: `7`
- `track_to_relocalize_emergency`: `7`

## Nejcastejsi forced replans

- `none`: `3600`

## Nejcastejsi forced relocalize

- `none`: `3590`
- `track_uncommitted`: `6`
- `false_goal_deadlock`: `2`
- `false_goal_offpath`: `1`
- `track_stall_false_goal`: `1`
