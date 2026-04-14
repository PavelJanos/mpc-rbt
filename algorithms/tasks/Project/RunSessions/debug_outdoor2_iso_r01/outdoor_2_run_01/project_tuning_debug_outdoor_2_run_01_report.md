# Project tuning debug

- mapa: `outdoor_2`
- repeat: `1`
- start: `[19.400, 13.600, 2.113]`
- cil: `[0.600, 1.000]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1677`
- final goal error: `0.500 m`
- travelled: `27.901 m`
- runtime: `178.731 s`

## Maxima a minima

- max XY localization error: `1.455 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.495`
- max ambiguity counter: `2`
- ambiguity active steps: `3`
- min front lidar: `0.964 m`
- max track stall steps: `13`
- max scan mismatch counter: `8`
- max map conflict counter: `0`
- max path revision: `5`
- max estimate jump: `0.722 m`
- max contradiction counter: `1`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `2`
- max hard path distance counter: `0`
- reseed attempts / successes: `3 / 3`
- max reseed crisis score: `0.543`
- max disambiguation goal switches: `2`
- min path quality score: `0.698`
- min trace bbox diag: `0.102 m`
- final estimated goal distance: `0.635 m`
- final true goal distance: `0.518 m`

## Stavy

- `commit`: `3` kroku
- `confirm`: `6` kroku
- `disambiguate`: `24` kroku
- `globalize`: `210` kroku
- `path_entry`: `30` kroku
- `plan`: `5` kroku
- `relocalize`: `140` kroku
- `track`: `1256` kroku
- `verify`: `3` kroku

## Nejcastejsi reasons

- `ok`: `1397`
- `fusion_disagreement_medium`: `262`
- `fusion_disagreement_large`: `10`
- `scan_match_very_bad`: `7`
- `scan_match_bad`: `1`

## Nejcastejsi transition reasons

- `stay`: `1644`
- `path_entry_to_track_merged`: `5`
- `plan_to_path_entry_path_ready`: `5`
- `confirm_to_globalize_drop`: `4`
- `commit_to_verify`: `3`
- `verify_to_plan`: `3`
- `localize_finish_to_commit`: `2`
- `localize_finish_to_confirm_fallback`: `2`

## Nejcastejsi forced replans

- `none`: `1677`

## Nejcastejsi forced relocalize

- `none`: `1677`
