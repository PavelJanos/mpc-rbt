# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `3600` kroku
- vysledek: `timeout`
- kroky: `3600`
- final goal error: `5.937 m`
- travelled: `42.690 m`
- runtime: `649.932 s`

## Maxima a minima

- max XY localization error: `12.059 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.020`
- max ambiguity counter: `63`
- ambiguity active steps: `1472`
- min front lidar: `0.461 m`
- max track stall steps: `10`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `9.753 m`
- max contradiction counter: `20`
- max no-progress counter: `19`
- max trace-loop counter: `7`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `1 / 1`
- max reseed crisis score: `0.039`
- max disambiguation goal switches: `2`
- min path quality score: `0.634`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `5.998 m`
- final true goal distance: `5.937 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `343` kroku
- `globalize`: `109` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `2827` kroku
- `track`: `315` kroku

## Nejcastejsi reasons

- `ok`: `1995`
- `pf_cluster_wide`: `960`
- `pf_cluster_very_wide`: `389`
- `pf_cluster_medium`: `180`
- `scan_match_very_bad`: `38`

## Nejcastejsi transition reasons

- `stay`: `3546`
- `relocalize_hard_reset_bad_hypothesis`: `20`
- `disambiguate_clear_to_relocalize`: `14`
- `relocalize_to_disambiguate_ambiguity`: `12`
- `localize_to_disambiguate_ambiguity`: `2`
- `commit_to_plan`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `path_entry_to_track_merged`: `1`

## Nejcastejsi forced replans

- `none`: `3600`

## Nejcastejsi forced relocalize

- `none`: `3599`
- `false_goal_uncommitted`: `1`
