# Project tuning debug

- mapa: `indoor_2`
- repeat: `9`
- start: `[3.200, 8.200, 1.642]`
- cil: `[7.000, 2.600]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `4.342 m`
- travelled: `23.945 m`
- runtime: `380.143 s`

## Maxima a minima

- max XY localization error: `11.213 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `50.818`
- max ambiguity counter: `64`
- ambiguity active steps: `863`
- min front lidar: `0.596 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `9.726 m`
- max contradiction counter: `20`
- max no-progress counter: `22`
- max trace-loop counter: `0`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `4.415 m`
- final true goal distance: `4.345 m`

## Stavy

- `disambiguate`: `189` kroku
- `globalize`: `91` kroku
- `relocalize`: `1770` kroku

## Nejcastejsi reasons

- `ok`: `898`
- `pf_cluster_wide`: `600`
- `pf_cluster_very_wide`: `430`
- `pf_cluster_medium`: `78`
- `scan_match_very_bad`: `25`

## Nejcastejsi transition reasons

- `stay`: `2022`
- `relocalize_hard_reset_bad_hypothesis`: `13`
- `disambiguate_clear_to_relocalize`: `7`
- `relocalize_to_disambiguate_ambiguity`: `6`
- `disambiguate_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
