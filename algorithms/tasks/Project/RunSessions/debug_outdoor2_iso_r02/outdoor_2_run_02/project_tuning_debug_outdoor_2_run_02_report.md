# Project tuning debug

- mapa: `outdoor_2`
- repeat: `2`
- start: `[0.600, 1.800, 1.464]`
- cil: `[19.400, 14.400]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1232`
- final goal error: `0.489 m`
- travelled: `24.142 m`
- runtime: `196.208 s`

## Maxima a minima

- max XY localization error: `1.455 m`
- max PF/EKF disagreement: `18.988 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.791`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.656 m`
- max track stall steps: `5`
- max scan mismatch counter: `9`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `0.722 m`
- max contradiction counter: `2`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `5`
- max hard path distance counter: `0`
- reseed attempts / successes: `6 / 6`
- max reseed crisis score: `0.931`
- max disambiguation goal switches: `2`
- min path quality score: `1.000`
- min trace bbox diag: `0.154 m`
- final estimated goal distance: `0.466 m`
- final true goal distance: `0.502 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `13` kroku
- `disambiguate`: `24` kroku
- `globalize`: `147` kroku
- `path_entry`: `8` kroku
- `plan`: `2` kroku
- `relocalize`: `33` kroku
- `track`: `1003` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1134`
- `fusion_disagreement_medium`: `94`
- `fusion_disagreement_large`: `4`

## Nejcastejsi transition reasons

- `stay`: `1219`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `commit_to_verify`: `1`
- `confirm_to_globalize_drop`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `localize_finish_to_commit`: `1`

## Nejcastejsi forced replans

- `none`: `1232`

## Nejcastejsi forced relocalize

- `none`: `1232`
