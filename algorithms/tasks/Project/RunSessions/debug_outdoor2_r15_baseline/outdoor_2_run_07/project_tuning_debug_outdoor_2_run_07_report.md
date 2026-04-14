# Project tuning debug

- mapa: `outdoor_2`
- repeat: `7`
- start: `[4.200, 0.600, -2.098]`
- cil: `[15.800, 0.600]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `13.413 m`
- travelled: `6.879 m`
- runtime: `1237.377 s`

## Maxima a minima

- max XY localization error: `15.987 m`
- max PF/EKF disagreement: `22.436 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.372`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `3.962 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `16.107 m`
- max contradiction counter: `20`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.055 m`
- final estimated goal distance: `13.621 m`
- final true goal distance: `13.406 m`

## Stavy

- `confirm`: `3` kroku
- `disambiguate`: `110` kroku
- `globalize`: `565` kroku
- `relocalize`: `1372` kroku

## Nejcastejsi reasons

- `fusion_disagreement_large`: `1227`
- `ok`: `654`
- `fusion_disagreement_medium`: `169`

## Nejcastejsi transition reasons

- `stay`: `2039`
- `confirm_to_globalize_drop`: `2`
- `disambiguate_clear_to_relocalize`: `2`
- `globalize_to_disambiguate_probe`: `2`
- `localize_hard_reset_bad_hypothesis`: `2`
- `relocalize_finish_to_confirm_fallback`: `2`
- `relocalize_hard_reset_bad_hypothesis`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
