# Project tuning debug

- mapa: `indoor_1`
- repeat: `1`
- start: `[9.200, 8.400, 0.265]`
- cil: `[0.800, 0.800]`
- timeout: `120` kroku
- vysledek: `timeout`
- kroky: `120`
- final goal error: `11.015 m`
- travelled: `0.542 m`
- runtime: `8.805 s`

## Maxima a minima

- max XY localization error: `11.060 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.345`
- max ambiguity counter: `6`
- ambiguity active steps: `12`
- min front lidar: `0.707 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `10.754 m`
- max contradiction counter: `1`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.047 m`
- final estimated goal distance: `11.019 m`
- final true goal distance: `11.022 m`

## Stavy

- `disambiguate`: `24` kroku
- `globalize`: `57` kroku
- `relocalize`: `39` kroku

## Nejcastejsi reasons

- `pf_cluster_wide`: `64`
- `ok`: `48`
- `pf_cluster_very_wide`: `4`
- `pf_cluster_medium`: `1`
- `pf_weakly_unique`: `1`

## Nejcastejsi transition reasons

- `stay`: `118`
- `disambiguate_clear_to_relocalize`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `120`

## Nejcastejsi forced relocalize

- `none`: `120`
