# Project tuning debug

- mapa: `indoor_1`
- repeat: `2`
- start: `[2.000, 0.800, -0.457]`
- cil: `[9.200, 9.200]`
- timeout: `1550` kroku
- vysledek: `goal`
- kroky: `1351`
- final goal error: `0.496 m`
- travelled: `23.721 m`
- runtime: `45.693 s`

## Maxima a minima

- max XY localization error: `11.514 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `8.842`
- max ambiguity counter: `110`
- ambiguity active steps: `326`
- min front lidar: `0.841 m`
- max track stall steps: `24`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `2`
- final estimated goal distance: `0.461 m`
- final true goal distance: `0.510 m`

## Stavy

- `localize`: `80` kroku
- `path_entry`: `57` kroku
- `plan`: `2` kroku
- `track`: `1212` kroku

## Nejcastejsi reasons

- `ok`: `1336`
- `scan_match_warn`: `4`
- `scan_match_bad`: `3`
- `pf_cluster_very_wide`: `2`
- `pf_not_unique`: `2`

## Nejcastejsi transition reasons

- `stay`: `1344`
- `localize_finish_to_plan`: `2`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `track_to_relocalize`: `1`

## Nejcastejsi forced replans

- `none`: `1351`

## Nejcastejsi forced relocalize

- `none`: `1287`
- `near_goal_loop`: `64`
