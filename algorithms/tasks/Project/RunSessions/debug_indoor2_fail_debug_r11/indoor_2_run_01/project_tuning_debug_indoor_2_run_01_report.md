# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `11.140 m`
- travelled: `24.629 m`
- runtime: `113.736 s`

## Maxima a minima

- max XY localization error: `11.036 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.487`
- max ambiguity counter: `57`
- ambiguity active steps: `920`
- min front lidar: `0.780 m`
- max track stall steps: `6`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `8.759 m`
- max contradiction counter: `20`
- max no-progress counter: `4`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `13 / 13`
- max reseed crisis score: `0.096`
- max disambiguation goal switches: `7`
- min path quality score: `0.834`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `0.749 m`
- final true goal distance: `11.123 m`

## Stavy

- `disambiguate`: `342` kroku
- `globalize`: `87` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `1445` kroku
- `track`: `171` kroku

## Nejcastejsi reasons

- `ok`: `1797`
- `pf_cluster_very_wide`: `131`
- `pf_cluster_medium`: `72`
- `pf_not_unique`: `24`
- `pf_cluster_wide`: `12`

## Nejcastejsi transition reasons

- `stay`: `2027`
- `relocalize_hard_reset_bad_hypothesis`: `9`
- `disambiguate_clear_to_relocalize`: `3`
- `disambiguate_hard_reset_bad_hypothesis`: `3`
- `relocalize_to_disambiguate_ambiguity`: `3`
- `disambiguate_force_exploratory_plan`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
