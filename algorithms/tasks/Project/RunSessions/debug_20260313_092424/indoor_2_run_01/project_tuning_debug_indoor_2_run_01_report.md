# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `5.257 m`
- travelled: `28.081 m`
- runtime: `146.492 s`

## Maxima a minima

- max XY localization error: `9.715 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `11.479`
- max ambiguity counter: `50`
- ambiguity active steps: `941`
- min front lidar: `0.832 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `7.323 m`
- max contradiction counter: `20`
- max no-progress counter: `19`
- max trace-loop counter: `10`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `5.243 m`
- final true goal distance: `5.248 m`

## Stavy

- `disambiguate`: `184` kroku
- `globalize`: `89` kroku
- `relocalize`: `1777` kroku

## Nejcastejsi reasons

- `ok`: `881`
- `pf_cluster_wide`: `564`
- `pf_cluster_very_wide`: `370`
- `pf_cluster_medium`: `167`
- `scan_match_very_bad`: `47`

## Nejcastejsi transition reasons

- `stay`: `2022`
- `relocalize_hard_reset_bad_hypothesis`: `12`
- `disambiguate_clear_to_relocalize`: `7`
- `relocalize_to_disambiguate_ambiguity`: `6`
- `disambiguate_hard_reset_bad_hypothesis`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
