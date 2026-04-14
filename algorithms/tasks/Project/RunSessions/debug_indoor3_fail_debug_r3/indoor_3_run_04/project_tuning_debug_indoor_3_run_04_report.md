# Project tuning debug

- mapa: `indoor_3`
- repeat: `4`
- start: `[9.200, 0.800, 0.337]`
- cil: `[3.600, 6.200]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `985`
- final goal error: `0.497 m`
- travelled: `19.810 m`
- runtime: `72.206 s`

## Maxima a minima

- max XY localization error: `10.007 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.985`
- max ambiguity counter: `50`
- ambiguity active steps: `272`
- min front lidar: `0.691 m`
- max track stall steps: `8`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `9.992 m`
- max contradiction counter: `14`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `2 / 2`
- max reseed crisis score: `0.100`
- max disambiguation goal switches: `1`
- min path quality score: `0.821`
- min trace bbox diag: `0.038 m`
- final estimated goal distance: `0.540 m`
- final true goal distance: `0.513 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `55` kroku
- `globalize`: `237` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `104` kroku
- `track`: `583` kroku

## Nejcastejsi reasons

- `ok`: `970`
- `pf_cluster_very_wide`: `5`
- `pf_cluster_medium`: `3`
- `scan_match_bad`: `3`
- `scan_match_warn`: `3`

## Nejcastejsi transition reasons

- `stay`: `978`
- `commit_to_plan`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`
- `relocalize_finish_to_commit`: `1`

## Nejcastejsi forced replans

- `none`: `985`

## Nejcastejsi forced relocalize

- `none`: `985`
