# Project tuning debug

- mapa: `outdoor_2`
- repeat: `1`
- start: `[19.400, 13.600, 2.113]`
- cil: `[0.600, 1.000]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1222`
- final goal error: `0.493 m`
- travelled: `25.576 m`
- runtime: `110.225 s`

## Maxima a minima

- max XY localization error: `1.455 m`
- max PF/EKF disagreement: `10.707 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.594`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.649 m`
- max track stall steps: `6`
- max scan mismatch counter: `8`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `0.721 m`
- max contradiction counter: `1`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `2`
- max hard path distance counter: `0`
- reseed attempts / successes: `2 / 2`
- max reseed crisis score: `0.525`
- max disambiguation goal switches: `1`
- min path quality score: `0.964`
- min trace bbox diag: `0.072 m`
- final estimated goal distance: `0.607 m`
- final true goal distance: `0.508 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `17` kroku
- `disambiguate`: `19` kroku
- `globalize`: `212` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `23` kroku
- `track`: `945` kroku

## Nejcastejsi reasons

- `ok`: `878`
- `fusion_disagreement_medium`: `286`
- `fusion_disagreement_large`: `58`

## Nejcastejsi transition reasons

- `stay`: `1205`
- `confirm_to_globalize_drop`: `5`
- `localize_finish_to_confirm_fallback`: `3`
- `localize_finish_to_confirm`: `2`
- `commit_to_plan`: `1`
- `confirm_to_disambiguate_probe`: `1`
- `disambiguate_finish_to_relocalize`: `1`
- `localize_finish_to_commit`: `1`

## Nejcastejsi forced replans

- `none`: `1222`

## Nejcastejsi forced relocalize

- `none`: `1222`
