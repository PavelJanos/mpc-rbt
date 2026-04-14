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
- repeats na mapu: `1`
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
| indoor_1 | 0.00 | 0 | 0 | 0 | 1 | NaN | 1.041 | 102.961 |
| indoor_2 | 0.00 | 0 | 0 | 0 | 1 | NaN | 11.323 | 231.890 |
| indoor_3 | 0.00 | 0 | 0 | 0 | 1 | NaN | 6.098 | 178.492 |
| mixed_1 | 0.00 | 0 | 0 | 1 | 0 | NaN | 18.207 | 390.126 |
| outdoor_1 | 0.00 | 0 | 0 | 0 | 1 | NaN | 3.504 | 283.119 |
| outdoor_2 | 1.00 | 1 | 0 | 0 | 0 | 1319.0 | 0.491 | 207.843 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_1 | 1 | 9.200 | 8.400 | 0.265 (0.084*pi) | 0.800 | 0.800 | timeout | 0 | 1600 | 1.041 | 26.991 | 102.961 |
| indoor_2 | 1 | 9.200 | 4.600 | 0.028 (0) | 0.800 | 0.800 | timeout | 0 | 1600 | 11.323 | 14.373 | 231.890 |
| indoor_3 | 1 | 0.800 | 9.200 | 1.989 (0.633*pi) | 0.800 | 0.800 | timeout | 0 | 1600 | 6.098 | 14.346 | 178.492 |
| mixed_1 | 1 | 19.400 | 14.400 | -1.605 (-pi/2) | 0.600 | 14.400 | out | 0 | 791 | 18.207 | 1.086 | 390.126 |
| outdoor_1 | 1 | 19.400 | 14.400 | 0.890 (0.283*pi) | 0.600 | 7.200 | timeout | 0 | 1600 | 3.504 | 25.387 | 283.119 |
| outdoor_2 | 1 | 19.400 | 0.600 | -2.531 (-0.806*pi) | 0.600 | 11.400 | goal | 1 | 1319 | 0.491 | 23.523 | 207.843 |
