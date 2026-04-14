# Project tuning debug

- mapa: `outdoor_2`
- repeat: `9`
- start: `[12.000, 0.600, 2.006]`
- cil: `[1.800, 4.000]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1565`
- final goal error: `0.499 m`
- travelled: `20.026 m`
- runtime: `740.750 s`

## Maxima a minima

- max XY localization error: `2.104 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.584`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `1.123 m`
- max track stall steps: `6`
- max scan mismatch counter: `3`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `1.198 m`
- max contradiction counter: `20`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `1`
- max hard path distance counter: `0`
- reseed attempts / successes: `34 / 34`
- max reseed crisis score: `1.225`
- max disambiguation goal switches: `1`
- min path quality score: `0.797`
- min trace bbox diag: `0.057 m`
- final estimated goal distance: `0.415 m`
- final true goal distance: `0.510 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `6` kroku
- `disambiguate`: `55` kroku
- `globalize`: `200` kroku
- `path_entry`: `6` kroku
- `plan`: `1` kroku
- `relocalize`: `638` kroku
- `track`: `658` kroku

## Nejcastejsi reasons

- `ok`: `989`
- `fusion_disagreement_large`: `557`
- `fusion_disagreement_medium`: `19`

## Nejcastejsi transition reasons

- `stay`: `1548`
- `confirm_to_globalize_drop`: `5`
- `localize_finish_to_confirm`: `3`
- `commit_to_plan`: `1`
- `confirm_to_disambiguate_probe`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `localize_finish_to_commit`: `1`
- `localize_finish_to_confirm_fallback`: `1`

## Nejcastejsi forced replans

- `none`: `1565`

## Nejcastejsi forced relocalize

- `none`: `1565`
