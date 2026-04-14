# Project tuning debug

- mapa: `indoor_1`
- repeat: `3`
- start: `[8.000, 6.000, 1.281]`
- cil: `[2.600, 3.000]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1157`
- final goal error: `0.496 m`
- travelled: `20.669 m`
- runtime: `76.294 s`

## Maxima a minima

- max XY localization error: `2.130 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.399`
- max ambiguity counter: `73`
- ambiguity active steps: `248`
- min front lidar: `1.131 m`
- max track stall steps: `181`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `3`
- max estimate jump: `1.748 m`
- max contradiction counter: `20`
- max no-progress counter: `9`
- max trace-loop counter: `3`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `0.090`
- max disambiguation goal switches: `5`
- min path quality score: `0.920`
- min trace bbox diag: `0.034 m`
- final estimated goal distance: `0.404 m`
- final true goal distance: `0.508 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `57` kroku
- `globalize`: `94` kroku
- `path_entry`: `12` kroku
- `plan`: `3` kroku
- `relocalize`: `348` kroku
- `track`: `641` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `872`
- `pf_cluster_wide`: `166`
- `pf_cluster_very_wide`: `108`
- `pf_cluster_medium`: `7`
- `pf_not_unique`: `2`

## Nejcastejsi transition reasons

- `stay`: `1138`
- `path_entry_to_track_merged`: `3`
- `plan_to_path_entry_path_ready`: `3`
- `relocalize_hard_reset_bad_hypothesis`: `3`
- `disambiguate_clear_to_relocalize`: `2`
- `commit_to_verify`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `1155`
- `away_from_goal`: `1`
- `track_stall`: `1`

## Nejcastejsi forced relocalize

- `none`: `1157`
