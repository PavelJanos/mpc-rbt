# Project tuning debug

- mapa: `indoor_2`
- repeat: `6`
- start: `[0.800, 9.200, 2.405]`
- cil: `[9.200, 6.400]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1101`
- final goal error: `0.495 m`
- travelled: `20.610 m`
- runtime: `51.321 s`

## Maxima a minima

- max XY localization error: `0.231 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.666`
- max ambiguity counter: `56`
- ambiguity active steps: `221`
- min front lidar: `0.608 m`
- max track stall steps: `10`
- max scan mismatch counter: `2`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `0.148 m`
- max contradiction counter: `0`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `0 / 0`
- max reseed crisis score: `0.000`
- max disambiguation goal switches: `0`
- min path quality score: `0.758`
- min trace bbox diag: `Inf m`
- final estimated goal distance: `0.374 m`
- final true goal distance: `0.503 m`

## Stavy

- `commit`: `1` kroku
- `globalize`: `164` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `track`: `931` kroku

## Nejcastejsi reasons

- `ok`: `1087`
- `scan_match_very_bad`: `6`
- `scan_match_warn`: `6`
- `pf_not_unique`: `1`
- `scan_match_bad`: `1`

## Nejcastejsi transition reasons

- `stay`: `1097`
- `commit_to_plan`: `1`
- `localize_finish_to_commit`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `1101`

## Nejcastejsi forced relocalize

- `none`: `1101`
