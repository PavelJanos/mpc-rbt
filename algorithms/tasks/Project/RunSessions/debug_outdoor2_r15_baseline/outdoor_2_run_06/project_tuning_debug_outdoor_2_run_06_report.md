# Project tuning debug

- mapa: `outdoor_2`
- repeat: `6`
- start: `[0.600, 9.400, -0.338]`
- cil: `[19.000, 10.800]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1614`
- final goal error: `0.478 m`
- travelled: `22.443 m`
- runtime: `232.895 s`

## Maxima a minima

- max XY localization error: `0.624 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `5.040`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.731 m`
- max track stall steps: `6`
- max scan mismatch counter: `8`
- max map conflict counter: `0`
- max path revision: `4`
- max estimate jump: `0.427 m`
- max contradiction counter: `4`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `3`
- max hard path distance counter: `0`
- reseed attempts / successes: `8 / 8`
- max reseed crisis score: `1.000`
- max disambiguation goal switches: `1`
- min path quality score: `1.000`
- min trace bbox diag: `0.040 m`
- final estimated goal distance: `0.689 m`
- final true goal distance: `0.500 m`

## Stavy

- `commit`: `4` kroku
- `confirm`: `30` kroku
- `disambiguate`: `55` kroku
- `globalize`: `282` kroku
- `path_entry`: `21` kroku
- `plan`: `4` kroku
- `relocalize`: `394` kroku
- `track`: `824` kroku

## Nejcastejsi reasons

- `ok`: `1518`
- `fusion_disagreement_large`: `47`
- `fusion_disagreement_medium`: `47`
- `scan_match_very_bad`: `2`

## Nejcastejsi transition reasons

- `stay`: `1585`
- `commit_to_plan`: `4`
- `path_entry_to_track_merged`: `4`
- `plan_to_path_entry_path_ready`: `4`
- `confirm_to_globalize_drop`: `3`
- `localize_finish_to_confirm`: `3`
- `track_to_relocalize`: `3`
- `localize_finish_to_commit`: `2`

## Nejcastejsi forced replans

- `none`: `1614`

## Nejcastejsi forced relocalize

- `none`: `1614`
