# Project tuning debug

- mapa: `indoor_2`
- repeat: `2`
- start: `[9.200, 8.600, 1.006]`
- cil: `[0.800, 3.800]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1425`
- final goal error: `0.499 m`
- travelled: `25.685 m`
- runtime: `82.194 s`

## Maxima a minima

- max XY localization error: `0.276 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `5.599`
- max ambiguity counter: `42`
- ambiguity active steps: `346`
- min front lidar: `0.660 m`
- max track stall steps: `21`
- max scan mismatch counter: `9`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `0.132 m`
- max contradiction counter: `20`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `1 / 1`
- max reseed crisis score: `0.084`
- max disambiguation goal switches: `0`
- min path quality score: `0.877`
- min trace bbox diag: `Inf m`
- final estimated goal distance: `0.511 m`
- final true goal distance: `0.515 m`

## Stavy

- `commit`: `1` kroku
- `globalize`: `245` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `track`: `1174` kroku

## Nejcastejsi reasons

- `ok`: `1371`
- `pf_cluster_medium`: `23`
- `scan_match_very_bad`: `14`
- `pf_not_unique`: `7`
- `pf_cluster_very_wide`: `6`

## Nejcastejsi transition reasons

- `stay`: `1420`
- `commit_to_plan`: `1`
- `localize_finish_to_commit`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `1425`

## Nejcastejsi forced relocalize

- `none`: `1425`
