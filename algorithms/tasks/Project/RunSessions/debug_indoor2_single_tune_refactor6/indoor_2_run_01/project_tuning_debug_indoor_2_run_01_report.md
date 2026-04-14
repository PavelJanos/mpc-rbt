# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `9.074 m`
- travelled: `3.339 m`
- runtime: `92.792 s`

## Maxima a minima

- max XY localization error: `8.090 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.224`
- max ambiguity counter: `17`
- ambiguity active steps: `54`
- min front lidar: `0.530 m`
- max track stall steps: `109`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `6.061 m`
- max contradiction counter: `17`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `1 / 1`
- max reseed crisis score: `0.039`
- max disambiguation goal switches: `2`
- min path quality score: `1.000`
- min trace bbox diag: `0.034 m`
- final estimated goal distance: `3.629 m`
- final true goal distance: `9.074 m`

## Stavy

- `commit`: `861` kroku
- `confirm`: `13` kroku
- `disambiguate`: `24` kroku
- `globalize`: `69` kroku
- `path_entry`: `5` kroku
- `plan`: `2` kroku
- `relocalize`: `919` kroku
- `track`: `156` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1976`
- `pf_cluster_wide`: `38`
- `pf_cluster_very_wide`: `32`
- `pf_weakly_unique`: `4`

## Nejcastejsi transition reasons

- `commit_gate_block`: `861`
- `relocalize_gate_to_commit`: `861`
- `stay`: `317`
- `plan_to_path_entry_path_ready`: `2`
- `confirm_to_verify`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `false_goal_deadlock`: `1`
