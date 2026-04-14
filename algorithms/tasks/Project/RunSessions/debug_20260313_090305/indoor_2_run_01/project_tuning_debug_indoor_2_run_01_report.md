# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `8.807 m`
- travelled: `18.130 m`
- runtime: `141.597 s`

## Maxima a minima

- max XY localization error: `9.432 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `11.264`
- max ambiguity counter: `35`
- ambiguity active steps: `696`
- min front lidar: `0.775 m`
- max track stall steps: `123`
- max scan mismatch counter: `4`
- max map conflict counter: `0`
- max path revision: `2`
- max estimate jump: `8.107 m`
- max contradiction counter: `20`
- max no-progress counter: `7`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `9 / 9`
- max reseed crisis score: `0.092`
- max disambiguation goal switches: `1`
- min path quality score: `1.000`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `4.447 m`
- final true goal distance: `8.802 m`

## Stavy

- `commit`: `2` kroku
- `disambiguate`: `55` kroku
- `globalize`: `81` kroku
- `path_entry`: `8` kroku
- `plan`: `2` kroku
- `relocalize`: `1456` kroku
- `track`: `444` kroku
- `verify`: `2` kroku

## Nejcastejsi reasons

- `ok`: `1435`
- `pf_cluster_wide`: `302`
- `pf_cluster_very_wide`: `204`
- `pf_cluster_medium`: `42`
- `scan_match_very_bad`: `26`

## Nejcastejsi transition reasons

- `stay`: `2028`
- `relocalize_hard_reset_bad_hypothesis`: `8`
- `commit_to_verify`: `2`
- `path_entry_to_track_merged`: `2`
- `plan_to_path_entry_path_ready`: `2`
- `relocalize_finish_to_commit`: `2`
- `verify_to_plan`: `2`
- `disambiguate_clear_to_relocalize`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
