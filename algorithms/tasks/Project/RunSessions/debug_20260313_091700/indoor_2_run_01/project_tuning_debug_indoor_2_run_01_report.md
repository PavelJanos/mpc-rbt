# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1338`
- final goal error: `0.492 m`
- travelled: `29.470 m`
- runtime: `79.184 s`

## Maxima a minima

- max XY localization error: `9.394 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.397`
- max ambiguity counter: `45`
- ambiguity active steps: `540`
- min front lidar: `0.579 m`
- max track stall steps: `45`
- max scan mismatch counter: `3`
- max map conflict counter: `0`
- max path revision: `3`
- max estimate jump: `9.438 m`
- max contradiction counter: `19`
- max no-progress counter: `22`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `4 / 4`
- max reseed crisis score: `0.234`
- max disambiguation goal switches: `1`
- min path quality score: `0.728`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `0.505 m`
- final true goal distance: `0.509 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `30` kroku
- `globalize`: `89` kroku
- `path_entry`: `12` kroku
- `plan`: `3` kroku
- `relocalize`: `298` kroku
- `track`: `904` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1045`
- `pf_cluster_wide`: `165`
- `pf_cluster_very_wide`: `100`
- `pf_cluster_medium`: `14`
- `scan_match_very_bad`: `6`

## Nejcastejsi transition reasons

- `stay`: `1321`
- `path_entry_to_track_merged`: `3`
- `plan_to_path_entry_path_ready`: `3`
- `relocalize_hard_reset_bad_hypothesis`: `3`
- `commit_to_verify`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `1337`
- `away_from_goal`: `1`

## Nejcastejsi forced relocalize

- `none`: `1338`
