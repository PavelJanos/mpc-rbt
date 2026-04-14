# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `9.829 m`
- travelled: `24.193 m`
- runtime: `147.444 s`

## Maxima a minima

- max XY localization error: `11.984 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.042`
- max ambiguity counter: `76`
- ambiguity active steps: `1048`
- min front lidar: `0.711 m`
- max track stall steps: `18`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `6.061 m`
- max contradiction counter: `20`
- max no-progress counter: `7`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `0.092`
- max disambiguation goal switches: `1`
- min path quality score: `0.802`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `3.165 m`
- final true goal distance: `9.826 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `346` kroku
- `globalize`: `81` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `1318` kroku
- `track`: `299` kroku

## Nejcastejsi reasons

- `ok`: `1037`
- `pf_cluster_wide`: `505`
- `pf_cluster_very_wide`: `406`
- `pf_cluster_medium`: `57`
- `scan_match_very_bad`: `16`

## Nejcastejsi transition reasons

- `stay`: `2023`
- `relocalize_hard_reset_bad_hypothesis`: `11`
- `disambiguate_clear_to_relocalize`: `4`
- `relocalize_to_disambiguate_ambiguity`: `3`
- `disambiguate_hard_reset_bad_hypothesis`: `2`
- `commit_to_plan`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `false_goal_deadlock`: `1`
