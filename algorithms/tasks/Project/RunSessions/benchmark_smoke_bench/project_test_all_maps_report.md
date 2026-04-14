# Project - Benchmark vsech map

Benchmark pouziva stejnou hlavni smycku jako `main`, ale bez GUI a bez `waitforbuttonpress`.

Pouzite nastaveni:
- planner: `A*`
- smoothing: `chaikin`
- controller: `pure pursuit`
- max velocity: `1.0 m/s`
- benchmark mode: `full`
- view mode: `true`
- stop mode: `all`
- repeats na mapu: `1`
- timeout: `50` kroku (`5.0 s` pri `Ts = 0.1 s`)
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
| indoor_1 | 0.00 | 0 | 0 | 0 | 1 | NaN | 11.851 | 4.985 |
| indoor_2 | 0.00 | 0 | 0 | 0 | 1 | NaN | 9.082 | 4.891 |
| indoor_3 | 0.00 | 0 | 0 | 0 | 1 | NaN | 8.496 | 4.971 |
| mixed_1 | 0.00 | 0 | 0 | 0 | 1 | NaN | 18.837 | 14.908 |
| outdoor_1 | 0.00 | 0 | 0 | 0 | 1 | NaN | 20.188 | 26.776 |
| outdoor_2 | 0.00 | 0 | 0 | 0 | 1 | NaN | 21.593 | 24.121 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_1 | 1 | 9.200 | 8.400 | 0.265 (0.084*pi) | 0.800 | 0.800 | timeout | 0 | 50 | 11.851 | 0.564 | 4.985 |
| indoor_2 | 1 | 9.200 | 4.600 | 0.028 (0) | 0.800 | 0.800 | timeout | 0 | 50 | 9.082 | 0.222 | 4.891 |
| indoor_3 | 1 | 0.800 | 9.200 | 1.989 (0.633*pi) | 0.800 | 0.800 | timeout | 0 | 50 | 8.496 | 0.109 | 4.971 |
| mixed_1 | 1 | 19.400 | 14.400 | -1.605 (-pi/2) | 0.600 | 14.400 | timeout | 0 | 50 | 18.837 | 0.097 | 14.908 |
| outdoor_1 | 1 | 19.400 | 14.400 | 0.890 (0.283*pi) | 0.600 | 7.200 | timeout | 0 | 50 | 20.188 | 0.061 | 26.776 |
| outdoor_2 | 1 | 19.400 | 0.600 | -2.531 (-0.806*pi) | 0.600 | 11.400 | timeout | 0 | 50 | 21.593 | 0.148 | 24.121 |
