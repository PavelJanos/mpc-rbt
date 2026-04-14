# Project tuning debug

- mapa: `outdoor_1`
- repeat: `1`
- start: `[19.400, 14.400, -1.362]`
- cil: `[0.600, 0.600]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1221`
- final goal error: `0.487 m`
- travelled: `25.805 m`
- runtime: `121.894 s`

## Maxima a minima

- max XY localization error: `1.209 m`
- max PF/EKF disagreement: `20.834 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.656`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.882 m`
- max track stall steps: `9`
- max scan mismatch counter: `5`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `0.572 m`
- max contradiction counter: `0`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `2`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `0.842`
- max disambiguation goal switches: `1`
- min path quality score: `0.982`
- min trace bbox diag: `0.140 m`
- final estimated goal distance: `0.778 m`
- final true goal distance: `0.507 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `8` kroku
- `disambiguate`: `50` kroku
- `globalize`: `121` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `track`: `1036` kroku

## Nejcastejsi reasons

- `ok`: `1054`
- `fusion_disagreement_large`: `143`
- `fusion_disagreement_medium`: `24`

## Nejcastejsi transition reasons

- `stay`: `1215`
- `commit_to_plan`: `1`
- `disambiguate_finish_to_confirm`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `localize_finish_to_commit`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `1221`

## Nejcastejsi forced relocalize

- `none`: `1221`
