# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `1600` kroku
- vysledek: `timeout`
- kroky: `1600`
- final goal error: `8.364 m`
- travelled: `17.928 m`
- runtime: `224.967 s`

## Maxima a minima

- max XY localization error: `9.190 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.269`
- max ambiguity counter: `37`
- ambiguity active steps: `423`
- min front lidar: `0.487 m`
- max track stall steps: `59`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `8.633 m`
- max contradiction counter: `20`
- max no-progress counter: `13`
- max trace-loop counter: `3`
- max commit watchdog counter: `0`
- max hard path distance counter: `8`
- reseed attempts / successes: `1 / 1`
- max reseed crisis score: `0.039`
- max disambiguation goal switches: `1`
- min path quality score: `1.000`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `8.385 m`
- final true goal distance: `8.370 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `73` kroku
- `globalize`: `69` kroku
- `path_entry`: `8` kroku
- `plan`: `2` kroku
- `relocalize`: `1171` kroku
- `track`: `275` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1065`
- `pf_cluster_wide`: `321`
- `pf_cluster_very_wide`: `127`
- `pf_cluster_medium`: `46`
- `scan_match_very_bad`: `21`

## Nejcastejsi transition reasons

- `stay`: `1579`
- `relocalize_hard_reset_bad_hypothesis`: `5`
- `disambiguate_clear_to_relocalize`: `3`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `relocalize_to_disambiguate_ambiguity`: `2`
- `commit_to_verify`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`

## Nejcastejsi forced replans

- `none`: `1600`

## Nejcastejsi forced relocalize

- `none`: `1599`
- `hard_path_distance`: `1`
