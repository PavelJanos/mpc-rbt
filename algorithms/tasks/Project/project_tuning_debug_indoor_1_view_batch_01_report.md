# Project tuning debug

- mapa: `indoor_1`
- repeat: `1`
- start: `[2.000, 0.800, 0.985]`
- cil: `[9.200, 9.200]`
- timeout: `1550` kroku
- vysledek: `goal`
- kroky: `891`
- final goal error: `0.494 m`
- travelled: `19.283 m`
- runtime: `32.312 s`

## Maxima a minima

- max XY localization error: `0.659 m`
- max PF/EKF disagreement: `Inf m`
- max PF dominant mass: `1.000`
- max PF top ratio: `4.149`
- max ambiguity counter: `113`
- ambiguity active steps: `164`
- min front lidar: `1.352 m`
- max track stall steps: `32`
- max scan mismatch counter: `1`
- max map conflict counter: `0`
- max path revision: `1`
- final estimated goal distance: `0.434 m`
- final true goal distance: `0.501 m`

## Stavy

- `localize`: `39` kroku
- `plan`: `1` kroku
- `track`: `851` kroku

## Nejcastejsi reasons

- `ok`: `883`
- `scan_match_bad`: `4`
- `scan_match_warn`: `3`
- `scan_match_very_bad`: `1`

## Nejcastejsi transition reasons

- `stay`: `889`
- `localize_finish_to_plan`: `1`
- `plan_to_track_path_ready`: `1`

## Nejcastejsi forced replans

- `none`: `891`

## Nejcastejsi forced relocalize

- `none`: `891`
