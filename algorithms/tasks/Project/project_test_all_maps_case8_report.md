# Project - Benchmark jedne mapy

Benchmark pouziva stejnou hlavni smycku jako `main`, ale bez GUI a bez `waitforbuttonpress`.

Pouzite nastaveni:
- planner: `A*`
- smoothing: `chaikin`
- controller: `pure pursuit`
- max velocity: `1.0 m/s`
- benchmark mode: `test`
- view mode: `true`
- selected test map: `indoor_1`
- repeats na mapu: `1`
- timeout: `1550` kroku (`155.0 s` pri `Ts = 0.1 s`)
- mode cases: `regenerate`
- start/cil/theta: nahodne generovane pro kazdy beh nebo znovu pouzite z CSV podle mode
- validace pripadu: clearance + existence cesty pres planner
- cases file: `project_test_all_maps_cases_test_indoor_1.csv`

## Konfigurace behu

| Mapa | Priklad start x [m] | Priklad start y [m] | Priklad start theta [rad] | Priklad cil x [m] | Priklad cil y [m] | Max vel [m/s] |
|---|---:|---:|---:|---:|---:|---:|
| indoor_1 | 8.200 | 0.800 | -1.955 (-0.622*pi) | 2.400 | 5.600 | 1.0 |
## Souhrn

| Mapa | Success rate | Goal | Wall | Out | Timeout | Mean steps (success) | Mean goal error [m] | Mean runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_1 | 1.00 | 1 | 0 | 0 | 0 | 697.0 | 0.485 | 24.027 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_1 | 1 | 8.200 | 0.800 | -1.955 (-0.622*pi) | 2.400 | 5.600 | goal | 1 | 697 | 0.485 | 11.436 | 24.027 |
