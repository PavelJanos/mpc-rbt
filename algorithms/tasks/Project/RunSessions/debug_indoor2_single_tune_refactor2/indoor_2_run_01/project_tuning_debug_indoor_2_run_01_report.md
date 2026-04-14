# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `5.195 m`
- travelled: `28.061 m`
- runtime: `420.178 s`

## Maxima a minima

- max XY localization error: `9.825 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.773`
- max ambiguity counter: `29`
- ambiguity active steps: `719`
- min front lidar: `0.871 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `7.453 m`
- max contradiction counter: `20`
- max no-progress counter: `20`
- max trace-loop counter: `7`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `5.187 m`
- final true goal distance: `5.195 m`

## Stavy

- `disambiguate`: `182` kroku
- `globalize`: `69` kroku
- `relocalize`: `1799` kroku

## Nejcastejsi reasons

- `ok`: `907`
- `pf_cluster_wide`: `674`
- `pf_cluster_very_wide`: `248`
- `pf_cluster_medium`: `155`
- `scan_match_very_bad`: `46`

## Nejcastejsi transition reasons

- `stay`: `2023`
- `relocalize_hard_reset_bad_hypothesis`: `11`
- `disambiguate_clear_to_relocalize`: `7`
- `relocalize_to_disambiguate_ambiguity`: `6`
- `disambiguate_hard_reset_bad_hypothesis`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
