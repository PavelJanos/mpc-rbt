# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `12.018 m`
- travelled: `7.710 m`
- runtime: `175.602 s`

## Maxima a minima

- max XY localization error: `12.226 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.089`
- max ambiguity counter: `25`
- ambiguity active steps: `638`
- min front lidar: `0.520 m`
- max track stall steps: `18`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `6.061 m`
- max contradiction counter: `17`
- max no-progress counter: `2`
- max trace-loop counter: `3`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `1 / 1`
- max reseed crisis score: `0.039`
- max disambiguation goal switches: `2`
- min path quality score: `0.616`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `2.110 m`
- final true goal distance: `12.018 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `48` kroku
- `globalize`: `232` kroku
- `path_entry`: `8` kroku
- `plan`: `2` kroku
- `relocalize`: `1417` kroku
- `track`: `341` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1796`
- `pf_cluster_wide`: `101`
- `pf_cluster_very_wide`: `84`
- `pf_cluster_medium`: `19`
- `scan_match_warn`: `16`

## Nejcastejsi transition reasons

- `stay`: `2036`
- `disambiguate_clear_to_relocalize`: `2`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `commit_to_verify`: `1`
- `confirm_to_disambiguate_probe`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2048`
- `false_goal_deadlock`: `2`
