# Project tuning debug

- mapa: `indoor_2`
- repeat: `9`
- start: `[3.200, 8.200, 1.642]`
- cil: `[7.000, 2.600]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `6.954 m`
- travelled: `23.545 m`
- runtime: `375.696 s`

## Maxima a minima

- max XY localization error: `11.213 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `68.878`
- max ambiguity counter: `64`
- ambiguity active steps: `889`
- min front lidar: `0.755 m`
- max track stall steps: `39`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `8.900 m`
- max contradiction counter: `20`
- max no-progress counter: `22`
- max trace-loop counter: `3`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `9 / 9`
- max reseed crisis score: `0.595`
- max disambiguation goal switches: `7`
- min path quality score: `1.000`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `6.885 m`
- final true goal distance: `6.954 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `190` kroku
- `globalize`: `118` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `1651` kroku
- `track`: `84` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1020`
- `pf_cluster_wide`: `629`
- `pf_cluster_very_wide`: `324`
- `pf_cluster_medium`: `27`
- `scan_match_very_bad`: `24`

## Nejcastejsi transition reasons

- `stay`: `2018`
- `relocalize_hard_reset_bad_hypothesis`: `12`
- `disambiguate_clear_to_relocalize`: `7`
- `relocalize_to_disambiguate_ambiguity`: `5`
- `localize_to_disambiguate_ambiguity`: `2`
- `commit_to_verify`: `1`
- `path_entry_to_track_merged`: `1`
- `plan_to_path_entry_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2049`
- `false_goal_deadlock`: `1`
