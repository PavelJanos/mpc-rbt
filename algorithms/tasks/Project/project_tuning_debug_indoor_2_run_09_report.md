# Project tuning debug

- mapa: `indoor_2`
- repeat: `9`
- start: `[3.200, 8.200, 1.642]`
- cil: `[7.000, 2.600]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `823`
- final goal error: `0.489 m`
- travelled: `16.927 m`
- runtime: `41.835 s`

## Maxima a minima

- max XY localization error: `4.502 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `39.799`
- max ambiguity counter: `29`
- ambiguity active steps: `233`
- min front lidar: `0.648 m`
- max track stall steps: `12`
- max scan mismatch counter: `5`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `4.593 m`
- max contradiction counter: `5`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `1 / 1`
- max reseed crisis score: `0.594`
- max disambiguation goal switches: `1`
- min path quality score: `0.899`
- min trace bbox diag: `0.062 m`
- final estimated goal distance: `0.488 m`
- final true goal distance: `0.505 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `46` kroku
- `globalize`: `121` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `track`: `650` kroku

## Nejcastejsi reasons

- `ok`: `786`
- `scan_match_very_bad`: `14`
- `pf_cluster_very_wide`: `9`
- `scan_match_warn`: `6`
- `scan_match_bad`: `4`

## Nejcastejsi transition reasons

- `stay`: `818`
- `commit_to_plan`: `1`
- `disambiguate_finish_to_commit`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `823`

## Nejcastejsi forced relocalize

- `none`: `823`
