# Project tuning debug

- mapa: `outdoor_2`
- repeat: `4`
- start: `[0.600, 14.200, 3.125]`
- cil: `[19.400, 2.800]`
- timeout: `2050` kroku
- vysledek: `out`
- kroky: `854`
- final goal error: `22.095 m`
- travelled: `1.243 m`
- runtime: `478.007 s`

## Maxima a minima

- max XY localization error: `1.209 m`
- max PF/EKF disagreement: `19.433 m`
- max PF dominant mass: `1.000`
- max PF top ratio: `44227035.145`
- max ambiguity counter: `0`
- ambiguity active steps: `0`
- min front lidar: `12.138 m`
- max track stall steps: `4`
- max scan mismatch counter: `4`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `0.572 m`
- max contradiction counter: `8`
- max no-progress counter: `0`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `6 / 6`
- max reseed crisis score: `1.158`
- max disambiguation goal switches: `0`
- min path quality score: `1.000`
- min trace bbox diag: `0.037 m`
- final estimated goal distance: `21.891 m`
- final true goal distance: `22.095 m`

## Stavy

- `commit`: `1` kroku
- `confirm`: `13` kroku
- `globalize`: `170` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `622` kroku
- `track`: `42` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `fusion_disagreement_large`: `796`
- `fusion_disagreement_medium`: `58`

## Nejcastejsi transition reasons

- `stay`: `841`
- `confirm_to_globalize_drop`: `3`
- `localize_finish_to_confirm_fallback`: `3`
- `commit_to_verify`: `1`
- `localize_finish_to_commit`: `1`
- `localize_finish_to_confirm`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `854`

## Nejcastejsi forced relocalize

- `none`: `854`
