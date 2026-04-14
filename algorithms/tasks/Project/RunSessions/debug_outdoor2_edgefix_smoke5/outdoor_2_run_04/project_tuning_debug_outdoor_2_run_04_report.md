# Project tuning debug

- mapa: `outdoor_2`
- repeat: `4`
- start: `[0.600, 14.200, 3.125]`
- cil: `[19.400, 2.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `22.727 m`
- travelled: `1.398 m`
- runtime: `587.154 s`

## Maxima a minima

- max XY localization error: `12.217 m`
- max PF/EKF disagreement: `19.952 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `452107306202.781`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `3.161 m`
- max track stall steps: `5`
- max scan mismatch counter: `3`
- max map conflict counter: `0`
- max path revision: `4`
- max estimate jump: `12.049 m`
- max contradiction counter: `20`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `8`
- reseed attempts / successes: `28 / 28`
- max reseed crisis score: `1.225`
- max disambiguation goal switches: `5`
- min path quality score: `1.000`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `22.944 m`
- final true goal distance: `22.727 m`

## Stavy

- `commit`: `3` kroku
- `confirm`: `58` kroku
- `disambiguate`: `120` kroku
- `globalize`: `1130` kroku
- `path_entry`: `56` kroku
- `plan`: `4` kroku
- `relocalize`: `553` kroku
- `track`: `123` kroku
- `verify`: `3` kroku

## Nejcastejsi reasons

- `fusion_disagreement_large`: `1958`
- `fusion_disagreement_medium`: `92`

## Nejcastejsi transition reasons

- `stay`: `1976`
- `confirm_to_globalize_drop`: `18`
- `localize_finish_to_confirm`: `6`
- `localize_finish_to_confirm_fallback`: `6`
- `confirm_to_disambiguate_probe`: `5`
- `disambiguate_clear_to_relocalize`: `5`
- `localize_hard_reset_bad_hypothesis`: `5`
- `path_entry_to_track_merged`: `4`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `hard_path_distance`: `1`
