# Project - Benchmark jedne mapy

Benchmark pouziva stejnou hlavni smycku jako `main`, ale bez GUI a bez `waitforbuttonpress`.

Pouzite nastaveni:
- planner: `A*`
- smoothing: `chaikin`
- controller: `pure pursuit`
- max velocity: `1.0 m/s`
- benchmark mode: `test`
- view mode: `false`
- stop mode: `all`
- selected test map: `indoor_2`
- repeats na mapu: `7`
- timeout: `2050` kroku (`205.0 s` pri `Ts = 0.1 s`)
- mode cases: `reuse`
- start/cil/theta: nahodne generovane pro kazdy beh nebo znovu pouzite z CSV podle mode
- validace pripadu: clearance + existence cesty pres planner
- cases file: `project_test_all_maps_cases_test_indoor_2.csv`

## Konfigurace behu

| Mapa | Priklad start x [m] | Priklad start y [m] | Priklad start theta [rad] | Priklad cil x [m] | Priklad cil y [m] | Max vel [m/s] |
|---|---:|---:|---:|---:|---:|---:|
| indoor_2 | 1.200 | 6.400 | -2.732 (-0.870*pi) | 7.200 | 5.000 | 1.0 |
## Souhrn

| Mapa | Success rate | Goal | Wall | Out | Timeout | Mean steps (success) | Mean goal error [m] | Mean runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_2 | 1.00 | 1 | 0 | 0 | 0 | 1392.0 | 0.496 | 176.038 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_2 | 7 | 1.200 | 6.400 | -2.732 (-0.870*pi) | 7.200 | 5.000 | goal | 1 | 1392 | 0.496 | 22.795 | 176.038 |
