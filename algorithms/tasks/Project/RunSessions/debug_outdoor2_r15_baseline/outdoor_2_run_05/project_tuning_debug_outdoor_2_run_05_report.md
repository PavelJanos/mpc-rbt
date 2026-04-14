# Project tuning debug

- mapa: `outdoor_2`
- repeat: `5`
- start: `[18.800, 11.000, 0.887]`
- cil: `[0.600, 11.400]`
- timeout: `2050` kroku
- vysledek: `out`
- kroky: `454`
- final goal error: `19.409 m`
- travelled: `1.523 m`
- runtime: `271.452 s`

## Maxima a minima

- max XY localization error: `1.318 m`
- max PF/EKF disagreement: `19.014 m`
- max PF dominant mass: `0.992`
- max PF top ratio: `1.983`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `11.772 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `1.185 m`
- max contradiction counter: `18`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.097 m`
- final estimated goal distance: `19.556 m`
- final true goal distance: `19.399 m`

## Stavy

- `confirm`: `6` kroku
- `disambiguate`: `55` kroku
- `globalize`: `153` kroku
- `relocalize`: `240` kroku

## Nejcastejsi reasons

- `fusion_disagreement_large`: `315`
- `ok`: `122`
- `fusion_disagreement_medium`: `17`

## Nejcastejsi transition reasons

- `stay`: `445`
- `confirm_to_globalize_drop`: `3`
- `localize_finish_to_confirm_fallback`: `2`
- `disambiguate_clear_to_relocalize`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `relocalize_finish_to_confirm_fallback`: `1`
- `relocalize_hard_reset_bad_hypothesis`: `1`

## Nejcastejsi forced replans

- `none`: `454`

## Nejcastejsi forced relocalize

- `none`: `454`
