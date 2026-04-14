# Project tuning debug

- mapa: `indoor_2`
- repeat: `3`
- start: `[1.200, 0.800, -2.576]`
- cil: `[9.200, 9.200]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1963`
- final goal error: `0.488 m`
- travelled: `32.099 m`
- runtime: `92.858 s`

## Maxima a minima

- max XY localization error: `11.575 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.657`
- max ambiguity counter: `98`
- ambiguity active steps: `790`
- min front lidar: `0.855 m`
- max track stall steps: `9`
- max scan mismatch counter: `2`
- max map conflict counter: `0`
- max path revision: `6`
- max estimate jump: `11.597 m`
- max contradiction counter: `20`
- max no-progress counter: `13`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `9 / 9`
- max reseed crisis score: `0.342`
- max disambiguation goal switches: `2`
- min path quality score: `0.656`
- min trace bbox diag: `0.037 m`
- final estimated goal distance: `0.551 m`
- final true goal distance: `0.505 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `110` kroku
- `globalize`: `79` kroku
- `path_entry`: `24` kroku
- `plan`: `6` kroku
- `relocalize`: `760` kroku
- `track`: `983` kroku

## Nejcastejsi reasons

- `ok`: `1701`
- `pf_cluster_very_wide`: `96`
- `pf_cluster_medium`: `85`
- `pf_not_unique`: `40`
- `pf_cluster_wide`: `23`

## Nejcastejsi transition reasons

- `stay`: `1934`
- `path_entry_to_track_merged`: `6`
- `plan_to_path_entry_path_ready`: `6`
- `relocalize_hard_reset_bad_hypothesis`: `6`
- `track_to_plan`: `5`
- `disambiguate_clear_to_relocalize`: `2`
- `commit_to_plan`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `1958`
- `away_from_goal`: `5`

## Nejcastejsi forced relocalize

- `none`: `1963`
