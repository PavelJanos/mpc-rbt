# Project tuning debug

- mapa: `outdoor_2`
- repeat: `3`
- start: `[19.400, 0.600, 0.733]`
- cil: `[0.600, 14.400]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1120`
- final goal error: `0.486 m`
- travelled: `24.432 m`
- runtime: `122.757 s`

## Maxima a minima

- max XY localization error: `0.656 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.244`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `3.117 m`
- max track stall steps: `12`
- max scan mismatch counter: `9`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `0.584 m`
- max contradiction counter: `1`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `3`
- max hard path distance counter: `0`
- reseed attempts / successes: `2 / 2`
- max reseed crisis score: `1.225`
- max disambiguation goal switches: `0`
- min path quality score: `0.759`
- min trace bbox diag: `Inf m`
- final estimated goal distance: `0.481 m`
- final true goal distance: `0.501 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `18` kroku
- `globalize`: `87` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `track`: `1009` kroku

## Nejcastejsi reasons

- `ok`: `1089`
- `fusion_disagreement_medium`: `15`
- `fusion_disagreement_large`: `9`
- `scan_match_very_bad`: `5`
- `scan_match_bad`: `1`

## Nejcastejsi transition reasons

- `stay`: `1111`
- `confirm_to_globalize_drop`: `2`
- `localize_finish_to_confirm_fallback`: `2`
- `commit_to_plan`: `1`
- `localize_finish_to_commit`: `1`
- `localize_finish_to_confirm`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `1120`

## Nejcastejsi forced relocalize

- `none`: `1120`
