# Project tuning debug

- mapa: `indoor_1`
- repeat: `6`
- start: `[9.200, 3.600, -0.456]`
- cil: `[4.800, 9.200]`
- timeout: `1550` kroku
- vysledek: `goal`
- kroky: `663`
- final goal error: `0.487 m`
- travelled: `11.949 m`
- runtime: `24.860 s`

## Maxima a minima

- max XY localization error: `10.021 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `172.575`
- max ambiguity counter: `25`
- ambiguity active steps: `58`
- min front lidar: `0.802 m`
- max track stall steps: `15`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `1`
- final estimated goal distance: `0.500 m`
- final true goal distance: `0.502 m`

## Stavy

- `localize`: `164` kroku
- `path_entry`: `28` kroku
- `plan`: `1` kroku
- `track`: `470` kroku

## Nejcastejsi reasons

- `ok`: `650`
- `scan_match_warn`: `5`
- `pf_not_unique`: `3`
- `scan_match_bad`: `3`
- `pf_cluster_very_wide`: `1`

## Nejcastejsi transition reasons

- `stay`: `660`
- `localize_finish_to_plan`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `663`

## Nejcastejsi forced relocalize

- `none`: `651`
- `near_goal_loop`: `12`
