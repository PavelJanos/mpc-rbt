# Project tuning debug

- mapa: `outdoor_1`
- repeat: `1`
- start: `[19.400, 14.400, -1.362]`
- cil: `[0.600, 0.600]`
- timeout: `1600` kroku
- vysledek: `timeout`
- kroky: `1600`
- final goal error: `3.087 m`
- travelled: `23.312 m`
- runtime: `345.529 s`

## Maxima a minima

- max XY localization error: `6.117 m`
- max PF/EKF disagreement: `20.834 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.320`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.882 m`
- max track stall steps: `9`
- max scan mismatch counter: `5`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `4.189 m`
- max contradiction counter: `16`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `6`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `0.842`
- max disambiguation goal switches: `1`
- min path quality score: `0.982`
- min trace bbox diag: `0.051 m`
- final estimated goal distance: `3.073 m`
- final true goal distance: `3.087 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `8` kroku
- `disambiguate`: `50` kroku
- `globalize`: `121` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `548` kroku
- `track`: `867` kroku

## Nejcastejsi reasons

- `ok`: `939`
- `fusion_disagreement_large`: `622`
- `fusion_disagreement_medium`: `39`

## Nejcastejsi transition reasons

- `stay`: `1592`
- `commit_to_plan`: `1`
- `disambiguate_finish_to_confirm`: `1`
- `globalize_to_disambiguate_probe`: `1`
- `localize_finish_to_commit`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`
- `relocalize_hard_reset_bad_hypothesis`: `1`

## Nejcastejsi forced replans

- `none`: `1600`

## Nejcastejsi forced relocalize

- `none`: `1599`
- `commit_watchdog`: `1`
