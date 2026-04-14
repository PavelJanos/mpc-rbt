# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `10.341 m`
- travelled: `17.352 m`
- runtime: `131.782 s`

## Maxima a minima

- max XY localization error: `11.046 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.487`
- max ambiguity counter: `49`
- ambiguity active steps: `701`
- min front lidar: `0.632 m`
- max track stall steps: `413`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `6.061 m`
- max contradiction counter: `20`
- max no-progress counter: `4`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `9 / 9`
- max reseed crisis score: `0.090`
- max disambiguation goal switches: `6`
- min path quality score: `1.000`
- min trace bbox diag: `0.036 m`
- final estimated goal distance: `2.094 m`
- final true goal distance: `10.355 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `359` kroku
- `globalize`: `87` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `1099` kroku
- `track`: `499` kroku

## Nejcastejsi reasons

- `ok`: `1817`
- `pf_cluster_very_wide`: `108`
- `pf_cluster_medium`: `80`
- `pf_cluster_wide`: `19`
- `pf_not_unique`: `16`

## Nejcastejsi transition reasons

- `stay`: `2026`
- `relocalize_hard_reset_bad_hypothesis`: `6`
- `disambiguate_clear_to_relocalize`: `4`
- `disambiguate_hard_reset_bad_hypothesis`: `4`
- `relocalize_to_disambiguate_ambiguity`: `3`
- `commit_to_plan`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `false_goal_deadlock`: `1`
