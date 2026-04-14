# Project tuning debug

- mapa: `indoor_1`
- repeat: `10`
- start: `[5.200, 0.800, -2.626]`
- cil: `[0.800, 7.000]`
- timeout: `1550` kroku
- vysledek: `goal`
- kroky: `1126`
- final goal error: `0.491 m`
- travelled: `18.066 m`
- runtime: `37.764 s`

## Maxima a minima

- max XY localization error: `9.224 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `7.142`
- max ambiguity counter: `53`
- ambiguity active steps: `280`
- min front lidar: `0.757 m`
- max track stall steps: `17`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `2`
- final estimated goal distance: `0.494 m`
- final true goal distance: `0.507 m`

## Stavy

- `localize`: `87` kroku
- `path_entry`: `79` kroku
- `plan`: `2` kroku
- `track`: `958` kroku

## Nejcastejsi reasons

- `ok`: `1099`
- `scan_match_warn`: `11`
- `scan_match_bad`: `6`
- `scan_match_very_bad`: `6`
- `pf_not_unique`: `3`

## Nejcastejsi transition reasons

- `stay`: `1119`
- `localize_finish_to_plan`: `2`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `track_to_relocalize`: `1`

## Nejcastejsi forced replans

- `none`: `1126`

## Nejcastejsi forced relocalize

- `none`: `1098`
- `near_goal_loop`: `28`
