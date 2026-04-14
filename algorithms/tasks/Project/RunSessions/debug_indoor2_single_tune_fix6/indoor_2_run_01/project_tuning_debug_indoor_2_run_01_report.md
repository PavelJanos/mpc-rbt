# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `7.085 m`
- travelled: `30.107 m`
- runtime: `329.460 s`

## Maxima a minima

- max XY localization error: `9.216 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `10.670`
- max ambiguity counter: `24`
- ambiguity active steps: `705`
- min front lidar: `0.704 m`
- max track stall steps: `99`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `7`
- max estimate jump: `7.736 m`
- max contradiction counter: `20`
- max no-progress counter: `22`
- max trace-loop counter: `5`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `12 / 12`
- max reseed crisis score: `0.730`
- max disambiguation goal switches: `12`
- min path quality score: `0.705`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `7.001 m`
- final true goal distance: `7.084 m`

## Stavy

- `commit`: `4` kroku
- `disambiguate`: `192` kroku
- `globalize`: `251` kroku
- `path_entry`: `28` kroku
- `plan`: `7` kroku
- `relocalize`: `1159` kroku
- `track`: `405` kroku
- `verify`: `4` kroku

## Nejcastejsi reasons

- `ok`: `1050`
- `pf_cluster_wide`: `544`
- `pf_cluster_very_wide`: `248`
- `pf_cluster_medium`: `168`
- `scan_match_very_bad`: `19`

## Nejcastejsi transition reasons

- `stay`: `1993`
- `disambiguate_clear_to_relocalize`: `8`
- `path_entry_to_track_merged`: `7`
- `plan_to_path_entry_path_ready`: `7`
- `relocalize_hard_reset_bad_hypothesis`: `7`
- `localize_to_disambiguate_ambiguity`: `5`
- `commit_to_verify`: `4`
- `relocalize_finish_to_commit`: `4`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2044`
- `track_uncommitted`: `4`
- `false_goal_deadlock`: `1`
- `false_goal_offpath`: `1`
