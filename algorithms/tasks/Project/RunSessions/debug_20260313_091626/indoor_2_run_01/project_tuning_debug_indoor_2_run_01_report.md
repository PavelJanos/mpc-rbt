# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1422`
- final goal error: `0.498 m`
- travelled: `30.582 m`
- runtime: `109.953 s`

## Maxima a minima

- max XY localization error: `9.394 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.403`
- max ambiguity counter: `46`
- ambiguity active steps: `681`
- min front lidar: `0.611 m`
- max track stall steps: `21`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `5`
- max estimate jump: `9.438 m`
- max contradiction counter: `19`
- max no-progress counter: `22`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `0.234`
- max disambiguation goal switches: `1`
- min path quality score: `0.772`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `0.493 m`
- final true goal distance: `0.513 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `30` kroku
- `globalize`: `89` kroku
- `path_entry`: `20` kroku
- `plan`: `5` kroku
- `relocalize`: `298` kroku
- `track`: `978` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1129`
- `pf_cluster_wide`: `165`
- `pf_cluster_very_wide`: `100`
- `pf_cluster_medium`: `14`
- `pf_weakly_unique`: `4`

## Nejcastejsi transition reasons

- `stay`: `1399`
- `path_entry_to_track_merged`: `5`
- `plan_to_path_entry_path_ready`: `5`
- `relocalize_hard_reset_bad_hypothesis`: `3`
- `track_to_plan_emergency`: `2`
- `commit_to_verify`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`

## Nejcastejsi forced replans

- `none`: `1419`
- `away_from_goal`: `3`

## Nejcastejsi forced relocalize

- `none`: `1422`
