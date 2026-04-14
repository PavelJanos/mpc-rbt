# Project tuning debug

- mapa: `indoor_1`
- repeat: `1`
- start: `[9.200, 8.400, 0.265]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1502`
- final goal error: `0.487 m`
- travelled: `23.796 m`
- runtime: `135.468 s`

## Maxima a minima

- max XY localization error: `11.060 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.345`
- max ambiguity counter: `156`
- ambiguity active steps: `567`
- min front lidar: `0.707 m`
- max track stall steps: `181`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `4`
- max estimate jump: `10.754 m`
- max contradiction counter: `20`
- max no-progress counter: `22`
- max trace-loop counter: `17`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `5 / 5`
- max reseed crisis score: `0.533`
- max disambiguation goal switches: `10`
- min path quality score: `0.731`
- min trace bbox diag: `0.036 m`
- final estimated goal distance: `0.482 m`
- final true goal distance: `0.503 m`

## Stavy

- `commit`: `2` kroku
- `disambiguate`: `116` kroku
- `globalize`: `57` kroku
- `path_entry`: `16` kroku
- `plan`: `4` kroku
- `relocalize`: `297` kroku
- `track`: `1008` kroku
- `verify`: `2` kroku

## Nejcastejsi reasons

- `ok`: `1170`
- `pf_cluster_wide`: `168`
- `pf_cluster_very_wide`: `154`
- `pf_cluster_medium`: `5`
- `pf_weakly_unique`: `2`

## Nejcastejsi transition reasons

- `stay`: `1475`
- `path_entry_to_track_merged`: `4`
- `plan_to_path_entry_path_ready`: `4`
- `disambiguate_clear_to_relocalize`: `3`
- `relocalize_hard_reset_bad_hypothesis`: `3`
- `commit_to_verify`: `2`
- `relocalize_finish_to_commit`: `2`
- `relocalize_to_disambiguate_ambiguity`: `2`

## Nejcastejsi forced replans

- `none`: `1500`
- `away_from_goal`: `1`
- `track_stall`: `1`

## Nejcastejsi forced relocalize

- `none`: `1502`
