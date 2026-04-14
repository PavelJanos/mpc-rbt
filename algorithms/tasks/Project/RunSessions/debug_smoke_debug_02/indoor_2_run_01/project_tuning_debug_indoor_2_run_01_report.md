# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[9.200, 4.600, 0.028]`
- cil: `[0.800, 0.800]`
- timeout: `60` kroku
- vysledek: `timeout`
- kroky: `60`
- final goal error: `9.140 m`
- travelled: `0.176 m`
- runtime: `6.080 s`

## Maxima a minima

- max XY localization error: `8.498 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.212`
- max ambiguity counter: `9`
- ambiguity active steps: `23`
- min front lidar: `0.751 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `8.501 m`
- max contradiction counter: `8`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `Inf m`
- final estimated goal distance: `9.106 m`
- final true goal distance: `9.146 m`

## Stavy

- `disambiguate`: `10` kroku
- `globalize`: `50` kroku

## Nejcastejsi reasons

- `pf_cluster_very_wide`: `33`
- `ok`: `26`
- `pf_not_unique`: `1`

## Nejcastejsi transition reasons

- `stay`: `59`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `60`

## Nejcastejsi forced relocalize

- `none`: `60`
