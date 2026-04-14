# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `5.452 m`
- travelled: `28.135 m`
- runtime: `304.077 s`

## Maxima a minima

- max XY localization error: `10.811 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.517`
- max ambiguity counter: `45`
- ambiguity active steps: `695`
- min front lidar: `0.342 m`
- max track stall steps: `42`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `3`
- max estimate jump: `8.065 m`
- max contradiction counter: `20`
- max no-progress counter: `20`
- max trace-loop counter: `18`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `9 / 9`
- max reseed crisis score: `0.115`
- max disambiguation goal switches: `13`
- min path quality score: `0.647`
- min trace bbox diag: `0.032 m`
- final estimated goal distance: `5.438 m`
- final true goal distance: `5.454 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `157` kroku
- `globalize`: `69` kroku
- `path_entry`: `12` kroku
- `plan`: `3` kroku
- `relocalize`: `1190` kroku
- `track`: `617` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1433`
- `pf_cluster_wide`: `278`
- `pf_cluster_very_wide`: `213`
- `pf_cluster_medium`: `91`
- `pf_not_unique`: `16`

## Nejcastejsi transition reasons

- `stay`: `2020`
- `relocalize_hard_reset_bad_hypothesis`: `7`
- `disambiguate_clear_to_relocalize`: `5`
- `relocalize_to_disambiguate_ambiguity`: `4`
- `path_entry_to_track_merged`: `3`
- `plan_to_path_entry_path_ready`: `3`
- `commit_to_verify`: `1`
- `disambiguate_hard_reset_bad_hypothesis`: `1`

## Nejcastejsi forced replans

- `none`: `2049`
- `away_from_goal`: `1`

## Nejcastejsi forced relocalize

- `none`: `2050`
