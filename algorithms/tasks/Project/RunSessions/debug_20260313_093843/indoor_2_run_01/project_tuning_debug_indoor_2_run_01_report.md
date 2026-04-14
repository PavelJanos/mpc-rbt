# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `1600` kroku
- vysledek: `timeout`
- kroky: `1600`
- final goal error: `9.613 m`
- travelled: `14.126 m`
- runtime: `208.775 s`

## Maxima a minima

- max XY localization error: `10.787 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `1.732`
- max ambiguity counter: `29`
- ambiguity active steps: `290`
- min front lidar: `0.390 m`
- max track stall steps: `181`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `3`
- max estimate jump: `6.061 m`
- max contradiction counter: `20`
- max no-progress counter: `5`
- max trace-loop counter: `0`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `2 / 2`
- max reseed crisis score: `0.093`
- max disambiguation goal switches: `2`
- min path quality score: `1.000`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `3.306 m`
- final true goal distance: `9.614 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `96` kroku
- `globalize`: `96` kroku
- `path_entry`: `12` kroku
- `plan`: `3` kroku
- `relocalize`: `696` kroku
- `track`: `695` kroku
- `verify`: `1` kroku

## Nejcastejsi reasons

- `ok`: `1192`
- `pf_cluster_wide`: `186`
- `pf_cluster_very_wide`: `152`
- `pf_cluster_medium`: `46`
- `pf_weakly_unique`: `12`

## Nejcastejsi transition reasons

- `stay`: `1575`
- `disambiguate_clear_to_relocalize`: `4`
- `relocalize_hard_reset_bad_hypothesis`: `4`
- `path_entry_to_track_merged`: `3`
- `plan_to_path_entry_path_ready`: `3`
- `localize_to_disambiguate_ambiguity`: `2`
- `relocalize_to_disambiguate_ambiguity`: `2`
- `commit_to_verify`: `1`

## Nejcastejsi forced replans

- `none`: `1599`
- `track_stall`: `1`

## Nejcastejsi forced relocalize

- `none`: `1599`
- `track_stall_false_goal`: `1`
