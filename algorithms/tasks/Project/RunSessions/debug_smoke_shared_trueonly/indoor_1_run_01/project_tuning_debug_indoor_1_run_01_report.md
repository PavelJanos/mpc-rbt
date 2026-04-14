# Project tuning debug

- mapa: `indoor_1`
- repeat: `1`
- start: `[9.200, 8.400, 0.265]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1022`
- final goal error: `0.486 m`
- travelled: `23.603 m`
- runtime: `39.727 s`

## Maxima a minima

- max XY localization error: `11.060 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.345`
- max ambiguity counter: `81`
- ambiguity active steps: `190`
- min front lidar: `0.808 m`
- max track stall steps: `12`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `10.761 m`
- max contradiction counter: `2`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `1 / 1`
- max reseed crisis score: `0.533`
- max disambiguation goal switches: `1`
- min path quality score: `0.589`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `0.545 m`
- final true goal distance: `0.501 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `43` kroku
- `globalize`: `161` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `track`: `812` kroku

## Nejcastejsi reasons

- `ok`: `999`
- `pf_cluster_medium`: `17`
- `pf_cluster_very_wide`: `3`
- `scan_match_bad`: `2`
- `scan_match_very_bad`: `1`

## Nejcastejsi transition reasons

- `stay`: `1017`
- `commit_to_plan`: `1`
- `disambiguate_finish_to_commit`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `1022`

## Nejcastejsi forced relocalize

- `none`: `1022`
