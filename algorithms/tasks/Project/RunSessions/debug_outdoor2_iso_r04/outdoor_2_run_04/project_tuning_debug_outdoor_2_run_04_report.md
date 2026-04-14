# Project tuning debug

- mapa: `outdoor_2`
- repeat: `4`
- start: `[0.600, 14.200, 3.125]`
- cil: `[19.400, 2.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `15.770 m`
- travelled: `16.954 m`
- runtime: `385.186 s`

## Maxima a minima

- max XY localization error: `6.063 m`
- max PF/EKF disagreement: `19.472 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `164519838907.523`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `1.013 m`
- max track stall steps: `6`
- max scan mismatch counter: `12`
- max map conflict counter: `0`
- max path revision: `7`
- max estimate jump: `5.996 m`
- max contradiction counter: `18`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `6`
- max hard path distance counter: `0`
- reseed attempts / successes: `12 / 12`
- max reseed crisis score: `1.158`
- max disambiguation goal switches: `5`
- min path quality score: `1.000`
- min trace bbox diag: `0.037 m`
- final estimated goal distance: `15.775 m`
- final true goal distance: `15.777 m`

## Stavy

- `commit`: `4` kroku
- `confirm`: `52` kroku
- `disambiguate`: `62` kroku
- `globalize`: `1265` kroku
- `path_entry`: `44` kroku
- `plan`: `7` kroku
- `relocalize`: `177` kroku
- `track`: `435` kroku
- `verify`: `4` kroku

## Nejcastejsi reasons

- `ok`: `1374`
- `fusion_disagreement_large`: `485`
- `fusion_disagreement_medium`: `191`

## Nejcastejsi transition reasons

- `stay`: `1972`
- `confirm_to_globalize_drop`: `17`
- `localize_finish_to_confirm_fallback`: `8`
- `path_entry_to_track_merged`: `7`
- `plan_to_path_entry_path_ready`: `7`
- `localize_finish_to_confirm`: `5`
- `commit_to_verify`: `4`
- `localize_hard_reset_bad_hypothesis`: `4`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2047`
- `commit_watchdog`: `3`
