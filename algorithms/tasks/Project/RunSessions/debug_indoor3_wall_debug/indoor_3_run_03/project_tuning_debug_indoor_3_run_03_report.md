# Project tuning debug

- mapa: `indoor_3`
- repeat: `3`
- start: `[2.400, 9.200, 1.642]`
- cil: `[8.800, 6.400]`
- timeout: `2050` kroku
- vysledek: `wall`
- kroky: `557`
- final goal error: `6.901 m`
- travelled: `7.798 m`
- runtime: `30.769 s`

## Maxima a minima

- max XY localization error: `0.747 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.432`
- max ambiguity counter: `32`
- ambiguity active steps: `136`
- min front lidar: `0.719 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `0.150 m`
- max contradiction counter: `20`
- max no-progress counter: `10`
- max trace-loop counter: `7`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `7.051 m`
- final true goal distance: `6.916 m`

## Stavy

- `disambiguate`: `55` kroku
- `globalize`: `139` kroku
- `relocalize`: `363` kroku

## Nejcastejsi reasons

- `ok`: `540`
- `pf_cluster_very_wide`: `6`
- `scan_match_warn`: `6`
- `pf_cluster_medium`: `2`
- `pf_cluster_wide`: `1`

## Nejcastejsi transition reasons

- `stay`: `552`
- `relocalize_hard_reset_bad_hypothesis`: `3`
- `disambiguate_clear_to_relocalize`: `1`
- `globalize_to_disambiguate_probe`: `1`

## Nejcastejsi forced replans

- `none`: `557`

## Nejcastejsi forced relocalize

- `none`: `557`
