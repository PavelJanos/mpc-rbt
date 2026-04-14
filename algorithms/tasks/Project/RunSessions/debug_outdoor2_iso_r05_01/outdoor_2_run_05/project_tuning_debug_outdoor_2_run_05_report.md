# Project tuning debug

- mapa: `outdoor_2`
- repeat: `5`
- start: `[18.800, 11.000, 0.887]`
- cil: `[0.600, 11.400]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `8.135 m`
- travelled: `27.234 m`
- runtime: `638.928 s`

## Maxima a minima

- max XY localization error: `2.752 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.205`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `1.391 m`
- max track stall steps: `4`
- max scan mismatch counter: `2`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `1.473 m`
- max contradiction counter: `14`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `3`
- max hard path distance counter: `0`
- reseed attempts / successes: `31 / 31`
- max reseed crisis score: `0.538`
- max disambiguation goal switches: `4`
- min path quality score: `0.657`
- min trace bbox diag: `0.061 m`
- final estimated goal distance: `8.268 m`
- final true goal distance: `8.178 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `6` kroku
- `disambiguate`: `48` kroku
- `globalize`: `841` kroku
- `path_entry`: `10` kroku
- `plan`: `2` kroku
- `relocalize`: `656` kroku
- `track`: `485` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1450`
- `fusion_disagreement_large`: `593`
- `fusion_disagreement_medium`: `7`

## Nejcastejsi transition reasons

- `stay`: `2029`
- `confirm_to_globalize_drop`: `4`
- `disambiguate_clear_to_relocalize`: `2`
- `localize_finish_to_confirm_fallback`: `2`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `commit_to_verify`: `1`
- `confirm_to_disambiguate_probe`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
