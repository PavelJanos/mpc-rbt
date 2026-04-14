# Project tuning debug

- mapa: `indoor_1`
- repeat: `1`
- start: `[9.200, 8.400, -2.882]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `982`
- final goal error: `0.486 m`
- travelled: `20.926 m`
- runtime: `46.835 s`

## Maxima a minima

- max XY localization error: `11.013 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.978`
- max ambiguity counter: `53`
- ambiguity active steps: `190`
- min front lidar: `1.364 m`
- max track stall steps: `31`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `10.885 m`
- max contradiction counter: `1`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `2 / 2`
- max reseed crisis score: `0.098`
- max disambiguation goal switches: `1`
- min path quality score: `0.769`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `0.508 m`
- final true goal distance: `0.502 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `55` kroku
- `globalize`: `33` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `31` kroku
- `track`: `857` kroku

## Nejcastejsi reasons

- `ok`: `954`
- `pf_cluster_very_wide`: `18`
- `pf_cluster_medium`: `8`
- `scan_match_very_bad`: `2`

## Nejcastejsi transition reasons

- `stay`: `976`
- `commit_to_plan`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `localize_to_disambiguate_ambiguity`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`
- `relocalize_finish_to_commit`: `1`

## Nejcastejsi forced replans

- `none`: `982`

## Nejcastejsi forced relocalize

- `none`: `982`
