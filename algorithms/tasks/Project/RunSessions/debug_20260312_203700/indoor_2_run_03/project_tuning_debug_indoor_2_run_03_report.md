# Project tuning debug

- mapa: `indoor_2`
- repeat: `3`
- start: `[1.200, 0.800, -2.576]`
- cil: `[9.200, 9.200]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `0.688 m`
- travelled: `31.467 m`
- runtime: `102.605 s`

## Maxima a minima

- max XY localization error: `11.477 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.905`
- max ambiguity counter: `119`
- ambiguity active steps: `874`
- min front lidar: `0.558 m`
- max track stall steps: `13`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `11.095 m`
- max contradiction counter: `20`
- max no-progress counter: `8`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `10 / 10`
- max reseed crisis score: `0.566`
- max disambiguation goal switches: `3`
- min path quality score: `0.841`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `0.676 m`
- final true goal distance: `0.699 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `145` kroku
- `globalize`: `120` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `705` kroku
- `track`: `1074` kroku

## Nejcastejsi reasons

- `ok`: `1811`
- `pf_cluster_very_wide`: `114`
- `pf_cluster_medium`: `76`
- `pf_cluster_wide`: `22`
- `scan_match_very_bad`: `8`

## Nejcastejsi transition reasons

- `stay`: `2034`
- `relocalize_hard_reset_bad_hypothesis`: `6`
- `disambiguate_clear_to_relocalize`: `2`
- `commit_to_plan`: `1`
- `disambiguate_hard_reset_bad_hypothesis`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`
- `path_entry_to_track_merged`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
