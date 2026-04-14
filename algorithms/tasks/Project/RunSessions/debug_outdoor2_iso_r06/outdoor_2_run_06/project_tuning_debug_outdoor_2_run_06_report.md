# Project tuning debug

- mapa: `outdoor_2`
- repeat: `6`
- start: `[0.600, 9.400, -0.338]`
- cil: `[19.000, 10.800]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1120`
- final goal error: `0.499 m`
- travelled: `20.662 m`
- runtime: `196.457 s`

## Maxima a minima

- max XY localization error: `1.455 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.939`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.645 m`
- max track stall steps: `7`
- max scan mismatch counter: `5`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `0.713 m`
- max contradiction counter: `2`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `4`
- max hard path distance counter: `0`
- reseed attempts / successes: `7 / 7`
- max reseed crisis score: `1.225`
- max disambiguation goal switches: `2`
- min path quality score: `1.000`
- min trace bbox diag: `0.118 m`
- final estimated goal distance: `0.497 m`
- final true goal distance: `0.515 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `17` kroku
- `disambiguate`: `16` kroku
- `globalize`: `158` kroku
- `path_entry`: `9` kroku
- `plan`: `2` kroku
- `track`: `916` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `999`
- `fusion_disagreement_large`: `106`
- `fusion_disagreement_medium`: `14`
- `scan_match_bad`: `1`

## Nejcastejsi transition reasons

- `stay`: `1104`
- `confirm_to_globalize_drop`: `3`
- `localize_finish_to_confirm_fallback`: `3`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `commit_to_verify`: `1`
- `disambiguate_finish_to_confirm`: `1`
- `globalize_to_disambiguate_probe`: `1`

## Nejcastejsi forced replans

- `none`: `1120`

## Nejcastejsi forced relocalize

- `none`: `1120`
