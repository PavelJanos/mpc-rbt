# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `wall`
- kroky: `1634`
- final goal error: `7.442 m`
- travelled: `17.818 m`
- runtime: `202.730 s`

## Maxima a minima

- max XY localization error: `10.811 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `7.189`
- max ambiguity counter: `51`
- ambiguity active steps: `561`
- min front lidar: `0.107 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `8.979 m`
- max contradiction counter: `20`
- max no-progress counter: `15`
- max trace-loop counter: `9`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `7.323 m`
- final true goal distance: `7.424 m`

## Stavy

- `disambiguate`: `124` kroku
- `globalize`: `69` kroku
- `relocalize`: `1441` kroku

## Nejcastejsi reasons

- `ok`: `934`
- `pf_cluster_wide`: `337`
- `pf_cluster_very_wide`: `258`
- `pf_cluster_medium`: `68`
- `pf_not_unique`: `17`

## Nejcastejsi transition reasons

- `stay`: `1615`
- `relocalize_hard_reset_bad_hypothesis`: `8`
- `disambiguate_clear_to_relocalize`: `5`
- `relocalize_to_disambiguate_ambiguity`: `4`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `1634`

## Nejcastejsi forced relocalize

- `none`: `1634`
