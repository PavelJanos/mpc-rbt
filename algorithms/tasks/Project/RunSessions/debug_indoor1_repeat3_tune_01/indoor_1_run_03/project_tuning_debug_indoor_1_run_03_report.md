# Project tuning debug

- mapa: `indoor_1`
- repeat: `3`
- start: `[8.000, 6.000, 1.281]`
- cil: `[2.600, 3.000]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1371`
- final goal error: `0.492 m`
- travelled: `23.804 m`
- runtime: `121.010 s`

## Maxima a minima

- max XY localization error: `2.130 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.947`
- max ambiguity counter: `73`
- ambiguity active steps: `331`
- min front lidar: `0.954 m`
- max track stall steps: `41`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `4`
- max estimate jump: `1.748 m`
- max contradiction counter: `20`
- max no-progress counter: `16`
- max trace-loop counter: `3`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `8 / 8`
- max reseed crisis score: `0.090`
- max disambiguation goal switches: `9`
- min path quality score: `0.609`
- min trace bbox diag: `0.033 m`
- final estimated goal distance: `0.486 m`
- final true goal distance: `0.508 m`

## Stavy

- `commit`: `3` kroku
- `disambiguate`: `105` kroku
- `globalize`: `289` kroku
- `path_entry`: `16` kroku
- `plan`: `4` kroku
- `relocalize`: `578` kroku
- `track`: `374` kroku
- `verify`: `2` kroku

## Nejcastejsi reasons

- `ok`: `794`
- `pf_cluster_wide`: `362`
- `pf_cluster_very_wide`: `186`
- `pf_cluster_medium`: `26`
- `pf_not_unique`: `2`

## Nejcastejsi transition reasons

- `stay`: `1337`
- `relocalize_hard_reset_bad_hypothesis`: `5`
- `disambiguate_clear_to_relocalize`: `4`
- `path_entry_to_track_merged`: `4`
- `plan_to_path_entry_path_ready`: `4`
- `relocalize_finish_to_commit`: `3`
- `track_to_relocalize_emergency`: `3`
- `commit_to_verify`: `2`

## Nejcastejsi forced replans

- `none`: `1371`

## Nejcastejsi forced relocalize

- `none`: `1367`
- `track_uncommitted`: `2`
- `false_goal_deadlock`: `1`
- `false_goal_uncommitted`: `1`
