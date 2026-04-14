# Project tuning debug

- mapa: `outdoor_1`
- repeat: `1`
- start: `[19.400, 14.400, -1.362]`
- cil: `[0.600, 0.600]`
- timeout: `300` kroku
- vysledek: `timeout`
- kroky: `300`
- final goal error: `21.781 m`
- travelled: `2.758 m`
- runtime: `38.708 s`

## Maxima a minima

- max XY localization error: `1.209 m`
- max PF/EKF disagreement: `20.898 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.594`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `2.451 m`
- max track stall steps: `8`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `0.572 m`
- max contradiction counter: `0`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `0.843`
- max disambiguation goal switches: `1`
- min path quality score: `0.982`
- min trace bbox diag: `0.066 m`
- final estimated goal distance: `21.978 m`
- final true goal distance: `21.800 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `6` kroku
- `disambiguate`: `39` kroku
- `globalize`: `121` kroku
- `path_entry`: `7` kroku
- `plan`: `1` kroku
- `track`: `125` kroku

## Nejcastejsi reasons

- `ok`: `154`
- `fusion_disagreement_large`: `132`
- `fusion_disagreement_medium`: `14`

## Nejcastejsi transition reasons

- `stay`: `294`
- `commit_to_plan`: `1`
- `disambiguate_finish_to_confirm`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `localize_finish_to_commit`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `300`

## Nejcastejsi forced relocalize

- `none`: `300`
