# Project tuning debug

- mapa: `outdoor_2`
- repeat: `8`
- start: `[15.600, 1.000, 0.627]`
- cil: `[3.800, 0.600]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1503`
- final goal error: `0.489 m`
- travelled: `26.186 m`
- runtime: `197.709 s`

## Maxima a minima

- max XY localization error: `0.605 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.063`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.609 m`
- max track stall steps: `17`
- max scan mismatch counter: `8`
- max map conflict counter: `0`
- max path revision: `4`
- max estimate jump: `0.406 m`
- max contradiction counter: `2`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `5`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `1.125`
- max disambiguation goal switches: `0`
- min path quality score: `0.653`
- min trace bbox diag: `0.191 m`
- final estimated goal distance: `0.594 m`
- final true goal distance: `0.508 m`

## Stavy

- `commit`: `2` kroku
- `confirm`: `15` kroku
- `globalize`: `373` kroku
- `path_entry`: `21` kroku
- `plan`: `4` kroku
- `relocalize`: `22` kroku
- `track`: `1064` kroku
- `verify`: `2` kroku

## Nejcastejsi reasons

- `ok`: `1447`
- `fusion_disagreement_large`: `39`
- `scan_match_very_bad`: `8`
- `fusion_disagreement_medium`: `5`
- `scan_match_bad`: `3`

## Nejcastejsi transition reasons

- `stay`: `1473`
- `confirm_to_globalize_drop`: `6`
- `localize_finish_to_confirm`: `5`
- `path_entry_to_track_merged`: `4`
- `plan_to_path_entry_path_ready`: `4`
- `commit_to_verify`: `2`
- `localize_finish_to_commit`: `2`
- `verify_complete_to_plan`: `2`

## Nejcastejsi forced replans

- `none`: `1503`

## Nejcastejsi forced relocalize

- `none`: `1503`
