# Project tuning debug

- mapa: `indoor_1`
- repeat: `2`
- start: `[2.000, 0.800, -0.457]`
- cil: `[9.200, 9.200]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `3.512 m`
- travelled: `39.748 m`
- runtime: `125.891 s`

## Maxima a minima

- max XY localization error: `0.897 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.803`
- max ambiguity counter: `57`
- ambiguity active steps: `294`
- min front lidar: `0.914 m`
- max track stall steps: `81`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `5`
- max estimate jump: `1.218 m`
- max contradiction counter: `20`
- max no-progress counter: `19`
- max trace-loop counter: `14`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `3 / 3`
- max reseed crisis score: `0.925`
- max disambiguation goal switches: `6`
- min path quality score: `0.789`
- min trace bbox diag: `0.035 m`
- final estimated goal distance: `3.563 m`
- final true goal distance: `3.496 m`

## Stavy

- `commit`: `3` kroku
- `disambiguate`: `138` kroku
- `globalize`: `448` kroku
- `path_entry`: `20` kroku
- `plan`: `5` kroku
- `relocalize`: `427` kroku
- `track`: `1006` kroku
- `verify`: `3` kroku

## Nejcastejsi reasons

- `ok`: `1563`
- `pf_cluster_wide`: `298`
- `pf_cluster_very_wide`: `149`
- `pf_cluster_medium`: `36`
- `scan_match_warn`: `2`

## Nejcastejsi transition reasons

- `stay`: `2013`
- `disambiguate_clear_to_relocalize`: `5`
- `path_entry_to_track_merged`: `5`
- `plan_to_path_entry_path_ready`: `5`
- `commit_to_verify`: `3`
- `relocalize_finish_to_commit`: `3`
- `relocalize_hard_reset_bad_hypothesis`: `3`
- `track_to_relocalize_emergency`: `3`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2046`
- `track_uncommitted`: `3`
- `false_goal_deadlock`: `1`
