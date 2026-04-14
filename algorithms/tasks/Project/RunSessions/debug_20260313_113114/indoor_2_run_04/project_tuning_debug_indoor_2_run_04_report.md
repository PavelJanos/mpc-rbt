# Project tuning debug

- mapa: `indoor_2`
- repeat: `4`
- start: `[9.200, 3.800, 1.671]`
- cil: `[0.800, 9.200]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1941`
- final goal error: `0.487 m`
- travelled: `29.841 m`
- runtime: `256.538 s`

## Maxima a minima

- max XY localization error: `8.874 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.811`
- max ambiguity counter: `89`
- ambiguity active steps: `674`
- min front lidar: `0.560 m`
- max track stall steps: `21`
- max scan mismatch counter: `5`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `8.266 m`
- max contradiction counter: `20`
- max no-progress counter: `9`
- max trace-loop counter: `5`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `8 / 8`
- max reseed crisis score: `0.088`
- max disambiguation goal switches: `14`
- min path quality score: `0.899`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `0.582 m`
- final true goal distance: `0.505 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `168` kroku
- `globalize`: `119` kroku
- `path_entry`: `8` kroku
- `plan`: `2` kroku
- `relocalize`: `963` kroku
- `track`: `679` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1379`
- `pf_cluster_wide`: `316`
- `pf_cluster_very_wide`: `172`
- `pf_cluster_medium`: `40`
- `pf_not_unique`: `12`

## Nejcastejsi transition reasons

- `stay`: `1916`
- `relocalize_hard_reset_bad_hypothesis`: `7`
- `disambiguate_clear_to_relocalize`: `5`
- `relocalize_to_disambiguate_ambiguity`: `4`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `commit_to_verify`: `1`
- `confirm_to_disambiguate_probe`: `1`

## Nejcastejsi forced replans

- `none`: `1941`

## Nejcastejsi forced relocalize

- `none`: `1940`
- `false_goal_deadlock`: `1`
