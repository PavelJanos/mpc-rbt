# Project tuning debug

- mapa: `outdoor_1`
- repeat: `1`
- start: `[19.400, 14.400, -1.362]`
- cil: `[0.600, 0.600]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `2.814 m`
- travelled: `25.657 m`
- runtime: `638.926 s`

## Maxima a minima

- max XY localization error: `1.209 m`
- max PF/EKF disagreement: `21.665 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.088`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.882 m`
- max track stall steps: `9`
- max scan mismatch counter: `5`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `0.572 m`
- max contradiction counter: `8`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `6`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `0.842`
- max disambiguation goal switches: `1`
- min path quality score: `0.982`
- min trace bbox diag: `0.140 m`
- final estimated goal distance: `2.718 m`
- final true goal distance: `2.814 m`

## Stavy

- `commit`: `2` kroku
- `confirm`: `8` kroku
- `disambiguate`: `50` kroku
- `globalize`: `121` kroku
- `path_entry`: `11` kroku
- `plan`: `2` kroku
- `relocalize`: `955` kroku
- `track`: `901` kroku

## Nejcastejsi reasons

- `ok`: `1188`
- `fusion_disagreement_large`: `818`
- `fusion_disagreement_medium`: `44`

## Nejcastejsi transition reasons

- `stay`: `2038`
- `commit_to_plan`: `2`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `track_to_relocalize_emergency`: `2`
- `disambiguate_finish_to_confirm`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `localize_finish_to_commit`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2048`
- `commit_watchdog`: `2`
