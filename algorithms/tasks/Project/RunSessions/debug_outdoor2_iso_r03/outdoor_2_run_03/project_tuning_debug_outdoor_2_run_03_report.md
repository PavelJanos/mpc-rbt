# Project tuning debug

- mapa: `outdoor_2`
- repeat: `3`
- start: `[19.400, 0.600, 0.733]`
- cil: `[0.600, 14.400]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `21.513 m`
- travelled: `15.250 m`
- runtime: `227.527 s`

## Maxima a minima

- max XY localization error: `1.455 m`
- max PF/EKF disagreement: `1.873 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.493`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `6.463 m`
- max track stall steps: `7`
- max scan mismatch counter: `8`
- max map conflict counter: `0`
- max path revision: `6`
- max estimate jump: `0.722 m`
- max contradiction counter: `4`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `6`
- max hard path distance counter: `0`
- reseed attempts / successes: `3 / 3`
- max reseed crisis score: `0.888`
- max disambiguation goal switches: `2`
- min path quality score: `1.000`
- min trace bbox diag: `0.060 m`
- final estimated goal distance: `21.695 m`
- final true goal distance: `21.511 m`

## Stavy

- `commit`: `3` kroku
- `confirm`: `347` kroku
- `disambiguate`: `24` kroku
- `globalize`: `957` kroku
- `path_entry`: `54` kroku
- `plan`: `6` kroku
- `relocalize`: `253` kroku
- `track`: `403` kroku
- `verify`: `3` kroku

## Nejcastejsi reasons

- `ok`: `1964`
- `fusion_disagreement_medium`: `85`
- `fusion_disagreement_large`: `1`

## Nejcastejsi transition reasons

- `stay`: `2007`
- `confirm_to_globalize_drop`: `7`
- `path_entry_to_track_merged`: `6`
- `plan_to_path_entry_path_ready`: `6`
- `localize_finish_to_confirm_fallback`: `4`
- `commit_to_verify`: `3`
- `verify_complete_to_plan`: `3`
- `verify_to_plan`: `3`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2048`
- `commit_watchdog`: `2`
