# Project tuning debug

- mapa: `indoor_3`
- repeat: `1`
- start: `[0.800, 3.800, -2.882]`
- cil: `[6.400, 5.000]`
- timeout: `1500` kroku
- vysledek: `goal`
- kroky: `1096`
- final goal error: `0.489 m`
- travelled: `20.998 m`
- runtime: `54.817 s`

## Maxima a minima

- max XY localization error: `2.782 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.274`
- max ambiguity counter: `61`
- ambiguity active steps: `255`
- min front lidar: `0.708 m`
- max track stall steps: `17`
- max scan mismatch counter: `2`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `2.665 m`
- max contradiction counter: `20`
- max no-progress counter: `6`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `2 / 2`
- max reseed crisis score: `0.064`
- max disambiguation goal switches: `1`
- min path quality score: `0.644`
- min trace bbox diag: `0.034 m`
- final estimated goal distance: `0.473 m`
- final true goal distance: `0.504 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `55` kroku
- `globalize`: `209` kroku
- `path_entry`: `8` kroku
- `plan`: `2` kroku
- `relocalize`: `260` kroku
- `track`: `561` kroku

## Nejcastejsi reasons

- `ok`: `1019`
- `pf_cluster_very_wide`: `50`
- `pf_cluster_medium`: `19`
- `scan_match_very_bad`: `4`
- `scan_match_warn`: `2`

## Nejcastejsi transition reasons

- `stay`: `1086`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `commit_to_plan`: `1`
- `confirm_to_disambiguate_probe`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `relocalize_finish_to_commit`: `1`
- `relocalize_hard_reset_bad_hypothesis`: `1`

## Nejcastejsi forced replans

- `none`: `1095`
- `away_from_goal`: `1`

## Nejcastejsi forced relocalize

- `none`: `1096`
