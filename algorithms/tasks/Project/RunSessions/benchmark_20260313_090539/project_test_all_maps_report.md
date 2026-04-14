# Project - Benchmark vsech map

Benchmark pouziva stejnou hlavni smycku jako `main`, ale bez GUI a bez `waitforbuttonpress`.

Pouzite nastaveni:
- planner: `A*`
- smoothing: `chaikin`
- controller: `pure pursuit`
- max velocity: `1.0 m/s`
- benchmark mode: `full`
- view mode: `false`
- stop mode: `all`
- repeats na mapu: `2`
- timeout: `1600` kroku (`160.0 s` pri `Ts = 0.1 s`)
- mode cases: `regenerate`
- start/cil/theta: nahodne generovane pro kazdy beh nebo znovu pouzite z CSV podle mode
- validace pripadu: clearance + existence cesty pres planner
- cases file: `project_test_all_maps_cases_full.csv`

## Konfigurace behu

| Mapa | Priklad start x [m] | Priklad start y [m] | Priklad start theta [rad] | Priklad cil x [m] | Priklad cil y [m] | Max vel [m/s] |
|---|---:|---:|---:|---:|---:|---:|
| indoor_1 | 9.200 | 8.400 | 0.265 (0.084*pi) | 0.800 | 0.800 | 1.0 |
| indoor_2 | 9.200 | 4.600 | 0.028 (0) | 0.800 | 0.800 | 1.0 |
| indoor_3 | 0.800 | 9.200 | 1.989 (0.633*pi) | 0.800 | 0.800 | 1.0 |
| mixed_1 | 19.400 | 14.400 | -1.605 (-pi/2) | 0.600 | 14.400 | 1.0 |
| outdoor_1 | 19.400 | 14.400 | 0.890 (0.283*pi) | 0.600 | 7.200 | 1.0 |
| outdoor_2 | 19.400 | 0.600 | -2.531 (-0.806*pi) | 0.600 | 11.400 | 1.0 |
## Souhrn

| Mapa | Success rate | Goal | Wall | Out | Timeout | Mean steps (success) | Mean goal error [m] | Mean runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_1 | 0.50 | 1 | 0 | 0 | 1 | 1507.0 | 3.635 | 82.839 |
| indoor_2 | 0.00 | 0 | 0 | 0 | 2 | NaN | 5.605 | 93.829 |
| indoor_3 | 0.00 | 0 | 0 | 0 | 2 | NaN | 5.646 | 105.801 |
| mixed_1 | 0.00 | 0 | 0 | 2 | 0 | NaN | 19.445 | 113.303 |
| outdoor_1 | 0.00 | 0 | 0 | 2 | 0 | NaN | 20.875 | 53.094 |
| outdoor_2 | 0.50 | 1 | 0 | 1 | 0 | 1233.0 | 10.813 | 105.789 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_1 | 1 | 9.200 | 8.400 | 0.265 (0.084*pi) | 0.800 | 0.800 | timeout | 0 | 1600 | 6.785 | 20.638 | 97.619 |
| indoor_1 | 2 | 2.000 | 0.800 | 2.258 (0.719*pi) | 9.200 | 9.200 | goal | 1 | 1507 | 0.485 | 28.080 | 68.059 |
| indoor_2 | 1 | 9.200 | 4.600 | 0.028 (0) | 0.800 | 0.800 | timeout | 0 | 1600 | 9.367 | 22.360 | 87.853 |
| indoor_2 | 2 | 9.200 | 9.000 | 0.198 (0.063*pi) | 0.800 | 5.000 | timeout | 0 | 1600 | 1.844 | 28.518 | 99.806 |
| indoor_3 | 1 | 0.800 | 9.200 | 1.989 (0.633*pi) | 0.800 | 0.800 | timeout | 0 | 1600 | 10.167 | 26.593 | 115.263 |
| indoor_3 | 2 | 3.600 | 9.200 | -2.792 (-0.889*pi) | 2.800 | 2.400 | timeout | 0 | 1600 | 1.125 | 30.352 | 96.338 |
| mixed_1 | 1 | 19.400 | 14.400 | -1.605 (-pi/2) | 0.600 | 14.400 | out | 0 | 203 | 19.416 | 2.161 | 99.737 |
| mixed_1 | 2 | 19.400 | 7.200 | -2.171 (-2*pi/3) | 0.600 | 6.800 | out | 0 | 265 | 19.474 | 3.103 | 126.868 |
| outdoor_1 | 1 | 19.400 | 14.400 | 0.890 (0.283*pi) | 0.600 | 7.200 | out | 0 | 46 | 20.813 | 0.696 | 46.623 |
| outdoor_1 | 2 | 19.400 | 6.800 | -0.267 (-0.085*pi) | 0.600 | 14.400 | out | 0 | 51 | 20.937 | 0.649 | 59.566 |
| outdoor_2 | 1 | 19.400 | 0.600 | -2.531 (-0.806*pi) | 0.600 | 11.400 | out | 0 | 86 | 21.134 | 1.157 | 14.999 |
| outdoor_2 | 2 | 19.400 | 8.400 | -2.492 (-0.793*pi) | 1.800 | 4.000 | goal | 1 | 1233 | 0.493 | 23.922 | 196.580 |
