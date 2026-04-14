# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `8.040 m`
- travelled: `24.443 m`
- runtime: `245.236 s`

## Maxima a minima

- max XY localization error: `10.200 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.269`
- max ambiguity counter: `44`
- ambiguity active steps: `688`
- min front lidar: `0.530 m`
- max track stall steps: `109`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `3`
- max estimate jump: `8.507 m`
- max contradiction counter: `20`
- max no-progress counter: `15`
- max trace-loop counter: `3`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `1 / 1`
- max reseed crisis score: `0.039`
- max disambiguation goal switches: `2`
- min path quality score: `1.000`
- min trace bbox diag: `0.034 m`
- final estimated goal distance: `8.090 m`
- final true goal distance: `8.058 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `120` kroku
- `disambiguate`: `192` kroku
- `globalize`: `561` kroku
- `path_entry`: `7` kroku
- `plan`: `3` kroku
- `relocalize`: `1009` kroku
- `track`: `156` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1072`
- `pf_cluster_wide`: `549`
- `pf_cluster_very_wide`: `321`
- `pf_cluster_medium`: `40`
- `scan_match_very_bad`: `32`

## Nejcastejsi transition reasons

- `stay`: `2005`
- `disambiguate_clear_to_relocalize`: `8`
- `relocalize_hard_reset_bad_hypothesis`: `7`
- `localize_hard_reset_bad_hypothesis`: `6`
- `localize_to_disambiguate_ambiguity`: `4`
- `relocalize_to_disambiguate_ambiguity`: `4`
- `plan_to_path_entry_path_ready`: `3`
- `relocalize_gate_to_confirm`: `3`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `false_goal_deadlock`: `1`
