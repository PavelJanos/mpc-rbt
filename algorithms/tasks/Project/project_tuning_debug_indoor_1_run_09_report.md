# Project tuning debug

- mapa: `indoor_1`
- repeat: `9`
- start: `[9.200, 3.600, -0.456]`
- cil: `[4.800, 9.200]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `3.493 m`
- travelled: `13.130 m`
- runtime: `78.546 s`

## Maxima a minima

- max XY localization error: `9.595 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `172.575`
- max ambiguity counter: `47`
- ambiguity active steps: `338`
- min front lidar: `0.548 m`
- max track stall steps: `565`
- max scan mismatch counter: `6`
- max map conflict counter: `0`
- max path revision: `7`
- max estimate jump: `5.517 m`
- max contradiction counter: `1`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `8`
- reseed attempts / successes: `2 / 2`
- max reseed crisis score: `0.099`
- max disambiguation goal switches: `1`
- min path quality score: `0.573`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `3.497 m`
- final true goal distance: `3.493 m`

## Stavy

- `disambiguate`: `55` kroku
- `localize`: `70` kroku
- `path_entry`: `25` kroku
- `plan`: `7` kroku
- `relocalize`: `189` kroku
- `track`: `1704` kroku

## Nejcastejsi reasons

- `ok`: `1961`
- `pf_cluster_very_wide`: `46`
- `scan_match_warn`: `16`
- `pf_not_unique`: `8`
- `scan_match_bad`: `8`

## Nejcastejsi transition reasons

- `stay`: `2025`
- `plan_to_path_entry_path_ready`: `7`
- `path_entry_to_track_merged`: `6`
- `relocalize_finish_to_plan`: `3`
- `track_to_plan`: `3`
- `track_to_relocalize_emergency`: `2`
- `disambiguate_clear_to_relocalize`: `1`
- `localize_finish_to_plan`: `1`

## Nejcastejsi forced replans

- `none`: `2047`
- `track_stall`: `3`

## Nejcastejsi forced relocalize

- `none`: `2047`
- `hard_path_distance`: `2`
- `near_goal_loop`: `1`
