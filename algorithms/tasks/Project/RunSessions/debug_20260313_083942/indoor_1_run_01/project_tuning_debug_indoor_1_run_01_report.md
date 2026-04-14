# Project tuning debug

- mapa: `indoor_1`
- repeat: `1`
- start: `[9.200, 8.400, -2.882]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `goal`
- kroky: `1694`
- final goal error: `0.493 m`
- travelled: `27.928 m`
- runtime: `77.681 s`

## Maxima a minima

- max XY localization error: `10.851 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `3.828`
- max ambiguity counter: `47`
- ambiguity active steps: `480`
- min front lidar: `0.989 m`
- max track stall steps: `4`
- max scan mismatch counter: `0`
- max map conflict counter: `0`
- max path revision: `1`
- max estimate jump: `9.759 m`
- max contradiction counter: `20`
- max no-progress counter: `11`
- max trace-loop counter: `9`
- max commit watchdog counter: `0`
- max hard path distance counter: `0`
- reseed attempts / successes: `8 / 8`
- max reseed crisis score: `0.076`
- max disambiguation goal switches: `2`
- min path quality score: `0.649`
- min trace bbox diag: `0.031 m`
- final estimated goal distance: `0.541 m`
- final true goal distance: `0.510 m`

## Stavy

- `commit`: `1` kroku
- `disambiguate`: `88` kroku
- `globalize`: `68` kroku
- `path_entry`: `4` kroku
- `plan`: `1` kroku
- `relocalize`: `1250` kroku
- `track`: `282` kroku

## Nejcastejsi reasons

- `ok`: `1607`
- `pf_cluster_very_wide`: `53`
- `pf_cluster_medium`: `16`
- `scan_match_warn`: `15`
- `pf_cluster_wide`: `3`

## Nejcastejsi transition reasons

- `stay`: `1680`
- `relocalize_hard_reset_bad_hypothesis`: `6`
- `commit_to_plan`: `1`
- `disambiguate_clear_to_relocalize`: `1`
- `disambiguate_hard_reset_bad_hypothesis`: `1`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`
- `path_entry_to_track_merged`: `1`

## Nejcastejsi forced replans

- `none`: `1694`

## Nejcastejsi forced relocalize

- `none`: `1694`
