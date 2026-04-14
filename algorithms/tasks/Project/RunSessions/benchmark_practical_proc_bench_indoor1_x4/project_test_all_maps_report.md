# Project - Benchmark jedne mapy

Benchmark pouziva stejnou hlavni smycku jako `main`, ale bez GUI a bez `waitforbuttonpress`.

Pouzite nastaveni:
- planner: `A*`
- smoothing: `chaikin`
- controller: `pure pursuit`
- max velocity: `1.0 m/s`
- benchmark mode: `test`
- view mode: `true`
- stop mode: `all`
- selected test map: `indoor_1`
- repeats na mapu: `4`
- timeout: `400` kroku (`40.0 s` pri `Ts = 0.1 s`)
- mode cases: `regenerate`
- start/cil/theta: nahodne generovane pro kazdy beh nebo znovu pouzite z CSV podle mode
- validace pripadu: clearance + existence cesty pres planner
- cases file: `project_test_all_maps_cases_test_indoor_1.csv`

## Konfigurace behu

| Mapa | Priklad start x [m] | Priklad start y [m] | Priklad start theta [rad] | Priklad cil x [m] | Priklad cil y [m] | Max vel [m/s] |
|---|---:|---:|---:|---:|---:|---:|
| indoor_1 | 2.000 | 0.800 | 2.258 (0.719*pi) | 9.200 | 9.200 | 1.0 |
## Souhrn

| Mapa | Success rate | Goal | Wall | Out | Timeout | Mean steps (success) | Mean goal error [m] | Mean runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_1 | 0.00 | 0 | 0 | 0 | 4 | NaN | 7.874 | 41.447 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_1 | 2 | 2.000 | 0.800 | 2.258 (0.719*pi) | 9.200 | 9.200 | timeout | 0 | 120 | 9.795 | 1.453 | 15.357 |
| indoor_1 | 3 | 8.000 | 6.000 | -1.923 (-0.612*pi) | 2.600 | 3.000 | timeout | 0 | 400 | 3.424 | 6.371 | 51.465 |
| indoor_1 | 4 | 9.200 | 3.600 | -1.863 (-0.593*pi) | 0.800 | 9.200 | timeout | 0 | 400 | 7.847 | 5.846 | 82.839 |
| indoor_1 | 1 | 9.200 | 8.400 | 0.265 (0.084*pi) | 0.800 | 0.800 | timeout | 0 | 120 | 10.429 | 1.013 | 16.129 |
