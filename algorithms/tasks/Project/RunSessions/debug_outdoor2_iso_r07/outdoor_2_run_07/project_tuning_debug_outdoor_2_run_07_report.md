# Project tuning debug

- mapa: `outdoor_2`
- repeat: `7`
- start: `[4.200, 0.600, -2.098]`
- cil: `[15.800, 0.600]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1215`
- final goal error: `0.498 m`
- travelled: `18.128 m`
- runtime: `295.353 s`

## Maxima a minima

- max XY localization error: `13.837 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.781`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `1.256 m`
- max track stall steps: `21`
- max scan mismatch counter: `9`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `13.658 m`
- max contradiction counter: `20`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `3`
- max hard path distance counter: `0`
- reseed attempts / successes: `9 / 9`
- max reseed crisis score: `1.106`
- max disambiguation goal switches: `4`
- min path quality score: `0.936`
- min trace bbox diag: `0.039 m`
- final estimated goal distance: `0.405 m`
- final true goal distance: `0.508 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `99` kroku
- `disambiguate`: `48` kroku
- `globalize`: `271` kroku
- `path_entry`: `16` kroku
- `plan`: `2` kroku
- `relocalize`: `101` kroku
- `track`: `676` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `861`
- `fusion_disagreement_large`: `316`
- `fusion_disagreement_medium`: `22`
- `scan_match_very_bad`: `8`
- `scan_match_bad`: `6`

## Nejcastejsi transition reasons

- `stay`: `1199`
- `disambiguate_clear_to_relocalize`: `2`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `commit_to_verify`: `1`
- `confirm_to_disambiguate_probe`: `1`
- `confirm_to_globalize_drop`: `1`
- `globalize_to_disambiguate_probe`: `1`

## Nejcastejsi forced replans

- `none`: `1215`

## Nejcastejsi forced relocalize

- `none`: `1215`
