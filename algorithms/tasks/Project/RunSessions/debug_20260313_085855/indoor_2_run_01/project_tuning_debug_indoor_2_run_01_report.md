# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `6.484 m`
- travelled: `26.361 m`
- runtime: `145.952 s`

## Maxima a minima

- max XY localization error: `12.046 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.042`
- max ambiguity counter: `120`
- ambiguity active steps: `948`
- min front lidar: `0.648 m`
- max track stall steps: `18`
- max scan mismatch counter: `2`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `8.705 m`
- max contradiction counter: `20`
- max no-progress counter: `12`
- max trace-loop counter: `5`
- max commit watchdog counter: `3`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `0.092`
- max disambiguation goal switches: `1`
- min path quality score: `0.701`
- min trace bbox diag: `0.033 m`
- final estimated goal distance: `6.562 m`
- final true goal distance: `6.476 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `220` kroku
- `globalize`: `203` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `1337` kroku
- `track`: `284` kroku

## Nejcastejsi reasons

- `ok`: `979`
- `pf_cluster_wide`: `583`
- `pf_cluster_very_wide`: `358`
- `pf_cluster_medium`: `70`
- `scan_match_very_bad`: `40`

## Nejcastejsi transition reasons

- `stay`: `2024`
- `relocalize_hard_reset_bad_hypothesis`: `12`
- `disambiguate_clear_to_relocalize`: `4`
- `relocalize_to_disambiguate_ambiguity`: `2`
- `commit_to_plan`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `false_goal_deadlock`: `1`
