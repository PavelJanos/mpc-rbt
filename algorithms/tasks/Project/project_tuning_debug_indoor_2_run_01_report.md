# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `1800` kroku
- vysledek: `goal`
- kroky: `1650`
- final goal error: `0.497 m`
- travelled: `27.118 m`
- runtime: `85.186 s`

## Maxima a minima

- max XY localization error: `9.132 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.655`
- max ambiguity counter: `53`
- ambiguity active steps: `696`
- min front lidar: `0.586 m`
- max track stall steps: `27`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `3`
- max estimate jump: `9.061 m`
- max contradiction counter: `20`
- max no-progress counter: `6`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `6 / 6`
- max reseed crisis score: `0.085`
- max disambiguation goal switches: `2`
- min path quality score: `0.784`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `0.530 m`
- final true goal distance: `0.509 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `95` kroku
- `globalize`: `39` kroku
- `path_entry`: `12` kroku
- `plan`: `3` kroku
- `relocalize`: `499` kroku
- `track`: `1001` kroku

## Nejcastejsi reasons

- `ok`: `1529`
- `pf_cluster_very_wide`: `95`
- `pf_cluster_medium`: `16`
- `scan_match_very_bad`: `5`
- `scan_match_bad`: `2`

## Nejcastejsi transition reasons

- `stay`: `1633`
- `relocalize_hard_reset_bad_hypothesis`: `4`
- `path_entry_to_track_merged`: `3`
- `plan_to_path_entry_path_ready`: `3`
- `commit_to_plan`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `disambiguate_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `1648`
- `away_from_goal`: `2`

## Nejcastejsi forced relocalize

- `none`: `1650`
