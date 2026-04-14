# Project tuning debug

- mapa: `outdoor_2`
- repeat: `10`
- start: `[0.600, 5.600, 1.522]`
- cil: `[19.400, 7.000]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `7.923 m`
- travelled: `18.064 m`
- runtime: `355.288 s`

## Maxima a minima

- max XY localization error: `2.530 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.306`
- max ambiguity counter: `1`
- ambiguity active steps: `1`
- min front lidar: `1.612 m`
- max track stall steps: `111`
- max scan mismatch counter: `3`
- max map conflict counter: `0`
- max path revision: `8`
- max estimate jump: `2.675 m`
- max contradiction counter: `20`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `6`
- max hard path distance counter: `8`
- reseed attempts / successes: `15 / 15`
- max reseed crisis score: `1.206`
- max disambiguation goal switches: `3`
- min path quality score: `1.000`
- min trace bbox diag: `0.034 m`
- final estimated goal distance: `7.935 m`
- final true goal distance: `7.916 m`

## Stavy

- `commit`: `8` kroku
- `confirm`: `201` kroku
- `disambiguate`: `91` kroku
- `globalize`: `623` kroku
- `path_entry`: `42` kroku
- `plan`: `8` kroku
- `relocalize`: `355` kroku
- `track`: `722` kroku

## Nejcastejsi reasons

- `ok`: `1748`
- `fusion_disagreement_large`: `218`
- `fusion_disagreement_medium`: `73`
- `scan_match_very_bad`: `9`
- `scan_match_bad`: `2`

## Nejcastejsi transition reasons

- `stay`: `1987`
- `commit_to_plan`: `8`
- `path_entry_to_track_merged`: `8`
- `plan_to_path_entry_path_ready`: `8`
- `track_to_relocalize_emergency`: `8`
- `confirm_to_globalize_drop`: `7`
- `localize_finish_to_commit`: `4`
- `relocalize_finish_to_commit`: `4`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2042`
- `commit_watchdog`: `5`
- `hard_path_distance`: `3`
