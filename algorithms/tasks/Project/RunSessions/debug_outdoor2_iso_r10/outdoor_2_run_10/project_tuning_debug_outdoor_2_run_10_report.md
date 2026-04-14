# Project tuning debug

- mapa: `outdoor_2`
- repeat: `10`
- start: `[0.600, 5.600, 1.522]`
- cil: `[19.400, 7.000]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1035`
- final goal error: `0.499 m`
- travelled: `20.278 m`
- runtime: `162.596 s`

## Maxima a minima

- max XY localization error: `1.455 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.538`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.676 m`
- max track stall steps: `8`
- max scan mismatch counter: `6`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `0.718 m`
- max contradiction counter: `2`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `5`
- max hard path distance counter: `0`
- reseed attempts / successes: `6 / 6`
- max reseed crisis score: `0.967`
- max disambiguation goal switches: `2`
- min path quality score: `1.000`
- min trace bbox diag: `0.085 m`
- final estimated goal distance: `0.344 m`
- final true goal distance: `0.509 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `24` kroku
- `globalize`: `119` kroku
- `path_entry`: `28` kroku
- `plan`: `2` kroku
- `relocalize`: `64` kroku
- `track`: `796` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `922`
- `fusion_disagreement_medium`: `110`
- `fusion_disagreement_large`: `2`
- `scan_match_very_bad`: `1`

## Nejcastejsi transition reasons

- `stay`: `1025`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `commit_to_verify`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `relocalize_finish_to_commit`: `1`
- `verify_complete_to_plan`: `1`

## Nejcastejsi forced replans

- `none`: `1035`

## Nejcastejsi forced relocalize

- `none`: `1035`
