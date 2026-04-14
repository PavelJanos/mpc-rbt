# Project tuning debug

- mapa: `indoor_2`
- repeat: `1`
- start: `[8.800, 6.200, -2.486]`
- cil: `[0.800, 0.800]`
- timeout: `2050` kroku
- vysledek: `timeout`
- kroky: `2050`
- final goal error: `8.206 m`
- travelled: `22.611 m`
- runtime: `112.635 s`

## Maxima a minima

- max XY localization error: `11.937 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `5.865`
- max ambiguity counter: `72`
- ambiguity active steps: `827`
- min front lidar: `0.757 m`
- max track stall steps: `NaN`
- max scan mismatch counter: `NaN`
- max map conflict counter: `NaN`
- max path revision: `0`
- max estimate jump: `7.747 m`
- max contradiction counter: `20`
- max no-progress counter: `22`
- max trace-loop counter: `12`
- max commit watchdog counter: `NaN`
- max hard path distance counter: `NaN`
- reseed attempts / successes: `NaN / NaN`
- max reseed crisis score: `NaN`
- max disambiguation goal switches: `NaN`
- min path quality score: `NaN`
- min trace bbox diag: `0.030 m`
- final estimated goal distance: `8.191 m`
- final true goal distance: `8.218 m`

## Stavy

- `disambiguate`: `312` kroku
- `globalize`: `87` kroku
- `relocalize`: `1651` kroku

## Nejcastejsi reasons

- `ok`: `1786`
- `pf_cluster_very_wide`: `137`
- `pf_cluster_medium`: `74`
- `pf_not_unique`: `16`
- `scan_match_very_bad`: `16`

## Nejcastejsi transition reasons

- `stay`: `2029`
- `relocalize_hard_reset_bad_hypothesis`: `10`
- `disambiguate_clear_to_relocalize`: `3`
- `disambiguate_hard_reset_bad_hypothesis`: `3`
- `relocalize_to_disambiguate_ambiguity`: `3`
- `localize_hard_reset_bad_hypothesis`: `1`
- `localize_to_disambiguate_ambiguity`: `1`

## Nejcastejsi forced replans

- `none`: `2050`

## Nejcastejsi forced relocalize

- `none`: `2050`
