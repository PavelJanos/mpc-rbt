# Project tuning debug

- mapa: `outdoor_2`
- repeat: `9`
- start: `[12.000, 0.600, 2.006]`
- cil: `[1.800, 4.000]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `4.864 m`
- travelled: `22.740 m`
- runtime: `751.342 s`

## Maxima a minima

- max XY localization error: `10.031 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.966`
- max ambiguity counter: `4`
- ambiguity active steps: `9`
- min front lidar: `0.293 m`
- max track stall steps: `13`
- max scan mismatch counter: `8`
- max map conflict counter: `0`
- max path revision: `4`
- max estimate jump: `12.404 m`
- max contradiction counter: `20`
- max no-progress counter: `2`
- max trace-loop counter: `0`
- max commit watchdog counter: `6`
- max hard path distance counter: `0`
- reseed attempts / successes: `25 / 25`
- max reseed crisis score: `1.225`
- max disambiguation goal switches: `4`
- min path quality score: `0.782`
- min trace bbox diag: `0.062 m`
- final estimated goal distance: `4.772 m`
- final true goal distance: `4.883 m`

## Stavy

- `commit`: `2` kroku
- `confirm`: `16` kroku
- `disambiguate`: `48` kroku
- `globalize`: `353` kroku
- `path_entry`: `36` kroku
- `plan`: `4` kroku
- `relocalize`: `1107` kroku
- `track`: `483` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `fusion_disagreement_large`: `1143`
- `ok`: `788`
- `pf_cluster_very_wide`: `105`
- `scan_match_very_bad`: `9`
- `scan_match_bad`: `4`

## Nejcastejsi transition reasons

- `stay`: `2015`
- `relocalize_hard_reset_bad_hypothesis`: `7`
- `path_entry_to_track_merged`: `4`
- `plan_to_path_entry_path_ready`: `4`
- `confirm_to_globalize_drop`: `3`
- `disambiguate_clear_to_relocalize`: `2`
- `globalize_to_disambiguate_probe`: `2`
- `localize_finish_to_commit`: `2`

## Nejcastejsi forced replans

- `none`: `2049`
- `track_uncommitted`: `1`

## Nejcastejsi forced relocalize

- `none`: `2048`
- `commit_watchdog`: `1`
- `track_uncommitted`: `1`
