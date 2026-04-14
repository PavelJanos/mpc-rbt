# Project tuning debug

- mapa: `outdoor_2`
- repeat: `3`
- start: `[19.400, 0.600, 0.733]`
- cil: `[0.600, 14.400]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1170`
- final goal error: `0.497 m`
- travelled: `23.689 m`
- runtime: `105.770 s`

## Maxima a minima

- max XY localization error: `1.455 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.493`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `1.187 m`
- max track stall steps: `7`
- max scan mismatch counter: `5`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `0.722 m`
- max contradiction counter: `0`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `1`
- max hard path distance counter: `0`
- reseed attempts / successes: `2 / 2`
- max reseed crisis score: `0.888`
- max disambiguation goal switches: `0`
- min path quality score: `0.641`
- min trace bbox diag: `Inf m`
- final estimated goal distance: `0.721 m`
- final true goal distance: `0.517 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `5` kroku
- `globalize`: `282` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `track`: `877` kroku

## Nejcastejsi reasons

- `ok`: `1080`
- `fusion_disagreement_medium`: `87`
- `scan_match_very_bad`: `2`
- `fusion_disagreement_large`: `1`

## Nejcastejsi transition reasons

- `stay`: `1160`
- `confirm_to_globalize_drop`: `3`
- `localize_finish_to_confirm_fallback`: `2`
- `commit_to_plan`: `1`
- `localize_finish_to_commit`: `1`
- `localize_finish_to_confirm`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `1170`

## Nejcastejsi forced relocalize

- `none`: `1170`
