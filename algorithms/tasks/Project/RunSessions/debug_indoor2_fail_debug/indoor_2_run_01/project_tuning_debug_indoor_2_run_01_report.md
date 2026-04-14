# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `8.959 m`
- travelled: `20.781 m`
- runtime: `99.911 s`

## Maxima a minima

- max XY localization error: `11.036 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.867`
- max ambiguity counter: `49`
- ambiguity active steps: `800`
- min front lidar: `0.780 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `8.759 m`
- max contradiction counter: `20`
- max no-progress counter: `6`
- max trace-loop counter: `0`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `4.848 m`
- final true goal distance: `8.959 m`

## Stavy

- `disambiguate`: `356` kroku
- `globalize`: `87` kroku
- `relocalize`: `1607` kroku

## Nejcastejsi reasons

- `ok`: `1779`
- `pf_cluster_very_wide`: `139`
- `pf_cluster_medium`: `77`
- `pf_not_unique`: `27`
- `pf_cluster_wide`: `12`

## Nejcastejsi transition reasons

- `stay`: `2028`
- `relocalize_hard_reset_bad_hypothesis`: `10`
- `disambiguate_clear_to_relocalize`: `4`
- `disambiguate_hard_reset_bad_hypothesis`: `3`
- `relocalize_to_disambiguate_ambiguity`: `3`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
