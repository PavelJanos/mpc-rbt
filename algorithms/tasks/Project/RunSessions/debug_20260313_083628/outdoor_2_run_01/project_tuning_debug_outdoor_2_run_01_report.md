# Project tuning debug

- mapa: `outdoor_2`
- repeat: `1`
- start: `[19.400, 0.600, -2.531]`
- cil: `[0.600, 11.400]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1033`
- final goal error: `0.496 m`
- travelled: `22.621 m`
- runtime: `89.358 s`

## Maxima a minima

- max XY localization error: `1.209 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.044`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `0.966 m`
- max track stall steps: `8`
- max scan mismatch counter: `13`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `0.572 m`
- max contradiction counter: `2`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `5`
- max hard path distance counter: `0`
- reseed attempts / successes: `2 / 2`
- max reseed crisis score: `0.700`
- max disambiguation goal switches: `0`
- min path quality score: `0.644`
- min trace bbox diag: `Inf m`
- final estimated goal distance: `0.497 m`
- final true goal distance: `0.502 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `27` kroku
- `globalize`: `80` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `track`: `920` kroku

## Nejcastejsi reasons

- `ok`: `863`
- `fusion_disagreement_large`: `126`
- `fusion_disagreement_medium`: `44`

## Nejcastejsi transition reasons

- `stay`: `1026`
- `localize_finish_to_confirm_fallback`: `2`
- `commit_to_plan`: `1`
- `confirm_to_globalize_drop`: `1`
- `localize_finish_to_commit`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `1033`

## Nejcastejsi forced relocalize

- `none`: `1033`
