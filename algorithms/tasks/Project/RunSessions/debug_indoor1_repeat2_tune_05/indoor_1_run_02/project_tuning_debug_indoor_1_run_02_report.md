# Project tuning debug

- mapa: `indoor_1`
- repeat: `2`
- start: `[2.000, 0.800, -0.457]`
- cil: `[9.200, 9.200]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1053`
- final goal error: `0.499 m`
- travelled: `21.482 m`
- runtime: `81.107 s`

## Maxima a minima

- max XY localization error: `0.897 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.410`
- max ambiguity counter: `46`
- ambiguity active steps: `140`
- min front lidar: `0.914 m`
- max track stall steps: `81`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `3`
- max estimate jump: `1.218 m`
- max contradiction counter: `0`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `1 / 1`
- max reseed crisis score: `0.925`
- max disambiguation goal switches: `2`
- min path quality score: `0.789`
- min trace bbox diag: `0.039 m`
- final estimated goal distance: `0.429 m`
- final true goal distance: `0.509 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `24` kroku
- `globalize`: `119` kroku
- `path_entry`: `12` kroku
- `plan`: `3` kroku
- `relocalize`: `13` kroku
- `track`: `880` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `981`
- `pf_cluster_wide`: `58`
- `pf_cluster_very_wide`: `10`
- `scan_match_warn`: `3`
- `scan_match_bad`: `1`

## Nejcastejsi transition reasons

- `stay`: `1040`
- `path_entry_to_track_merged`: `3`
- `plan_to_path_entry_path_ready`: `3`
- `commit_to_verify`: `1`
- `confirm_to_disambiguate_probe`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `relocalize_finish_to_commit`: `1`
- `track_to_plan`: `1`

## Nejcastejsi forced replans

- `none`: `1051`
- `false_goal_deadlock`: `1`
- `track_uncommitted`: `1`

## Nejcastejsi forced relocalize

- `none`: `1053`
