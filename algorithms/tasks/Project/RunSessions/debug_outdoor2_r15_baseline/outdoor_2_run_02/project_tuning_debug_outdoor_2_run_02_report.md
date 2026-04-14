# Project tuning debug

- mapa: `outdoor_2`
- repeat: `2`
- start: `[0.600, 1.800, 1.464]`
- cil: `[19.400, 14.400]`
- timeout: `2050` kroku
- vysledek: `out`
- kroky: `400`
- final goal error: `22.324 m`
- travelled: `2.087 m`
- runtime: `83.093 s`

## Maxima a minima

- max XY localization error: `0.498 m`
- max PF/EKF disagreement: `17.298 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.625`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `8.790 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `0.409 m`
- max contradiction counter: `7`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.034 m`
- final estimated goal distance: `22.322 m`
- final true goal distance: `22.315 m`

## Stavy

- `confirm`: `6` kroku
- `disambiguate`: `55` kroku
- `globalize`: `217` kroku
- `relocalize`: `122` kroku

## Nejcastejsi reasons

- `ok`: `213`
- `fusion_disagreement_medium`: `147`
- `fusion_disagreement_large`: `40`

## Nejcastejsi transition reasons

- `stay`: `392`
- `confirm_to_globalize_drop`: `3`
- `localize_finish_to_confirm_fallback`: `3`
- `disambiguate_clear_to_relocalize`: `1`
- `globalize_to_disambiguate_probe`: `1`

## Nejcastejsi forced replans

- `none`: `400`

## Nejcastejsi forced relocalize

- `none`: `400`
