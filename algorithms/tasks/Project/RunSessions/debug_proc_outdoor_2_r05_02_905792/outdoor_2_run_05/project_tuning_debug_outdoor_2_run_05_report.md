# Project tuning debug

- mapa: `outdoor_2`
- repeat: `5`
- start: `[18.800, 11.000, 0.887]`
- cil: `[0.600, 11.400]`
- timeout: `2050` kroku
- vysledek: `wall`
- kroky: `1765`
- final goal error: `12.666 m`
- travelled: `22.794 m`
- runtime: `525.575 s`

## Maxima a minima

- max XY localization error: `1.209 m`
- max PF/EKF disagreement: `18.454 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `2.628`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.307 m`
- max track stall steps: `7`
- max scan mismatch counter: `6`
- max map conflict counter: `0`
- max path revision: `3`
- max estimate jump: `0.605 m`
- max contradiction counter: `20`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `6`
- max hard path distance counter: `0`
- reseed attempts / successes: `26 / 26`
- max reseed crisis score: `0.985`
- max disambiguation goal switches: `2`
- min path quality score: `0.617`
- min trace bbox diag: `0.057 m`
- final estimated goal distance: `12.346 m`
- final true goal distance: `12.693 m`

## Stavy

- `commit`: `2` kroku
- `confirm`: `15` kroku
- `disambiguate`: `24` kroku
- `globalize`: `178` kroku
- `path_entry`: `35` kroku
- `plan`: `3` kroku
- `relocalize`: `830` kroku
- `track`: `676` kroku
- `verify`: `2` kroku

## Nejcastejsi reasons

- `ok`: `1223`
- `fusion_disagreement_large`: `533`
- `fusion_disagreement_medium`: `9`

## Nejcastejsi transition reasons

- `stay`: `1741`
- `confirm_to_globalize_drop`: `3`
- `path_entry_to_track_merged`: `3`
- `plan_to_path_entry_path_ready`: `3`
- `commit_to_verify`: `2`
- `localize_finish_to_confirm_fallback`: `2`
- `verify_to_plan`: `2`
- `disambiguate_clear_to_relocalize`: `1`

## Nejcastejsi forced replans

- `none`: `1765`

## Nejcastejsi forced relocalize

- `none`: `1764`
- `commit_watchdog`: `1`
