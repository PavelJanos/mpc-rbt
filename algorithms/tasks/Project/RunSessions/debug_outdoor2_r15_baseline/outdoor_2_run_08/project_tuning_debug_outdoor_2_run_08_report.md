# Project tuning debug

- mapa: `outdoor_2`
- repeat: `8`
- start: `[15.600, 1.000, 0.627]`
- cil: `[3.800, 0.600]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `2.282 m`
- travelled: `24.405 m`
- runtime: `446.715 s`

## Maxima a minima

- max XY localization error: `0.606 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.734`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.906 m`
- max track stall steps: `12`
- max scan mismatch counter: `8`
- max map conflict counter: `0`
- max path revision: `5`
- max estimate jump: `0.507 m`
- max contradiction counter: `5`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `6`
- max hard path distance counter: `0`
- reseed attempts / successes: `6 / 6`
- max reseed crisis score: `0.769`
- max disambiguation goal switches: `1`
- min path quality score: `0.668`
- min trace bbox diag: `0.062 m`
- final estimated goal distance: `2.290 m`
- final true goal distance: `2.282 m`

## Stavy

- `commit`: `5` kroku
- `confirm`: `2` kroku
- `disambiguate`: `24` kroku
- `globalize`: `303` kroku
- `path_entry`: `21` kroku
- `plan`: `5` kroku
- `relocalize`: `857` kroku
- `track`: `833` kroku

## Nejcastejsi reasons

- `ok`: `1873`
- `fusion_disagreement_large`: `167`
- `scan_match_very_bad`: `5`
- `scan_match_bad`: `2`
- `scan_match_warn`: `2`

## Nejcastejsi transition reasons

- `stay`: `2019`
- `commit_to_plan`: `5`
- `path_entry_to_track_merged`: `5`
- `plan_to_path_entry_path_ready`: `5`
- `track_to_relocalize_emergency`: `5`
- `relocalize_finish_to_commit`: `4`
- `confirm_to_globalize_drop`: `2`
- `disambiguate_finish_to_relocalize`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2045`
- `commit_watchdog`: `5`
