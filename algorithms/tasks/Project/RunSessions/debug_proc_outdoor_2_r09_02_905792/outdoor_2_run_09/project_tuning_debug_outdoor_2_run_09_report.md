# Project tuning debug

- mapa: `outdoor_2`
- repeat: `9`
- start: `[12.000, 0.600, 2.006]`
- cil: `[1.800, 4.000]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `9.465 m`
- travelled: `20.088 m`
- runtime: `885.328 s`

## Maxima a minima

- max XY localization error: `6.334 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.966`
- max ambiguity counter: `4`
- ambiguity active steps: `9`
- min front lidar: `0.293 m`
- max track stall steps: `13`
- max scan mismatch counter: `7`
- max map conflict counter: `0`
- max path revision: `3`
- max estimate jump: `3.328 m`
- max contradiction counter: `20`
- max no-progress counter: `2`
- max trace-loop counter: `0`
- max commit watchdog counter: `1`
- max hard path distance counter: `0`
- reseed attempts / successes: `22 / 22`
- max reseed crisis score: `0.970`
- max disambiguation goal switches: `4`
- min path quality score: `0.754`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `9.519 m`
- final true goal distance: `9.466 m`

## Stavy

- `commit`: `2` kroku
- `confirm`: `6` kroku
- `disambiguate`: `48` kroku
- `globalize`: `353` kroku
- `path_entry`: `13` kroku
- `plan`: `3` kroku
- `relocalize`: `1117` kroku
- `track`: `506` kroku
- `verify`: `2` kroku

## Nejcastejsi reasons

- `fusion_disagreement_large`: `1082`
- `ok`: `881`
- `pf_cluster_very_wide`: `72`
- `scan_match_very_bad`: `9`
- `fusion_disagreement_medium`: `6`

## Nejcastejsi transition reasons

- `stay`: `2019`
- `relocalize_hard_reset_bad_hypothesis`: `7`
- `confirm_to_globalize_drop`: `3`
- `path_entry_to_track_merged`: `3`
- `plan_to_path_entry_path_ready`: `3`
- `commit_to_verify`: `2`
- `disambiguate_clear_to_relocalize`: `2`
- `globalize_to_disambiguate_probe`: `2`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `track_uncommitted`: `1`
