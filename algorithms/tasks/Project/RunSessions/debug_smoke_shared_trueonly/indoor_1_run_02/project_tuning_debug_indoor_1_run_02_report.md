# Project tuning debug

- mapa: `indoor_1`
- repeat: `2`
- start: `[2.000, 0.800, 2.258]`
- cil: `[9.200, 9.200]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1548`
- final goal error: `0.492 m`
- travelled: `25.357 m`
- runtime: `61.872 s`

## Maxima a minima

- max XY localization error: `0.466 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.968`
- max ambiguity counter: `117`
- ambiguity active steps: `676`
- min front lidar: `1.140 m`
- max track stall steps: `5`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `0.402 m`
- max contradiction counter: `20`
- max no-progress counter: `6`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `10 / 10`
- max reseed crisis score: `0.078`
- max disambiguation goal switches: `1`
- min path quality score: `0.624`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `0.508 m`
- final true goal distance: `0.508 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `55` kroku
- `globalize`: `64` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `954` kroku
- `track`: `469` kroku

## Nejcastejsi reasons

- `ok`: `1432`
- `pf_cluster_very_wide`: `99`
- `pf_cluster_medium`: `9`
- `scan_match_very_bad`: `3`
- `scan_match_warn`: `2`

## Nejcastejsi transition reasons

- `stay`: `1532`
- `relocalize_hard_reset_bad_hypothesis`: `9`
- `commit_to_plan`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `1548`

## Nejcastejsi forced relocalize

- `none`: `1548`
