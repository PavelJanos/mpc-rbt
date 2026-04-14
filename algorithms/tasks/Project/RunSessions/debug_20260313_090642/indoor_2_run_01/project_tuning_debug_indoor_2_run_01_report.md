# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `wall`
- kroky: `584`
- final goal error: `7.450 m`
- travelled: `9.715 m`
- runtime: `37.370 s`

## Maxima a minima

- max XY localization error: `9.567 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.587`
- max ambiguity counter: `38`
- ambiguity active steps: `233`
- min front lidar: `0.121 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `9.472 m`
- max contradiction counter: `17`
- max no-progress counter: `22`
- max trace-loop counter: `12`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `7.353 m`
- final true goal distance: `7.427 m`

## Stavy

- `disambiguate`: `27` kroku
- `globalize`: `85` kroku
- `relocalize`: `472` kroku

## Nejcastejsi reasons

- `pf_cluster_wide`: `209`
- `pf_cluster_very_wide`: `195`
- `ok`: `136`
- `pf_cluster_medium`: `25`
- `scan_match_very_bad`: `8`

## Nejcastejsi transition reasons

- `stay`: `576`
- `relocalize_hard_reset_bad_hypothesis`: `5`
- `disambiguate_clear_to_relocalize`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `584`

## Nejcastejsi forced relocalize

- `none`: `584`
