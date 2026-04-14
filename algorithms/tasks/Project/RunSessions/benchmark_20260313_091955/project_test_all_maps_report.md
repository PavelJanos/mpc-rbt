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
- selected test map: `indoor_2`
- repeats na mapu: `10`
- timeout: `2050` kroku (`205.0 s` pri `Ts = 0.1 s`)
- mode cases: `regenerate`
- start/cil/theta: nahodne generovane pro kazdy beh nebo znovu pouzite z CSV podle mode
- validace pripadu: clearance + existence cesty pres planner
- cases file: `project_test_all_maps_cases_test_indoor_2.csv`

## Konfigurace behu

| Mapa | Priklad start x [m] | Priklad start y [m] | Priklad start theta [rad] | Priklad cil x [m] | Priklad cil y [m] | Max vel [m/s] |
|---|---:|---:|---:|---:|---:|---:|
| indoor_2 | 8.800 | 6.200 | -2.486 (-0.791*pi) | 0.800 | 0.800 | 1.0 |
## Souhrn

| Mapa | Success rate | Goal | Wall | Out | Timeout | Mean steps (success) | Mean goal error [m] | Mean runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_2 | 0.30 | 3 | 1 | 0 | 6 | 1742.0 | 4.673 | 129.873 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_2 | 1 | 8.800 | 6.200 | -2.486 (-0.791*pi) | 0.800 | 0.800 | timeout | 0 | 2050 | 9.507 | 18.078 | 119.872 |
| indoor_2 | 2 | 9.200 | 8.600 | 1.006 (pi/3) | 0.800 | 3.800 | timeout | 0 | 2050 | 3.286 | 27.410 | 134.615 |
| indoor_2 | 3 | 1.200 | 0.800 | -2.576 (-0.820*pi) | 9.200 | 9.200 | timeout | 0 | 2050 | 10.057 | 20.405 | 181.109 |
| indoor_2 | 4 | 9.200 | 3.800 | 1.671 (0.532*pi) | 0.800 | 9.200 | timeout | 0 | 2050 | 5.101 | 25.461 | 144.029 |
| indoor_2 | 5 | 0.800 | 3.800 | 2.155 (2*pi/3) | 9.200 | 3.400 | timeout | 0 | 2050 | 5.103 | 31.163 | 151.400 |
| indoor_2 | 6 | 0.800 | 9.200 | 2.405 (3*pi/4) | 9.200 | 6.400 | goal | 1 | 1849 | 0.491 | 31.771 | 132.563 |
| indoor_2 | 7 | 1.200 | 6.400 | -2.732 (-0.870*pi) | 7.200 | 5.000 | timeout | 0 | 2050 | 5.484 | 26.822 | 187.156 |
| indoor_2 | 8 | 7.000 | 2.800 | -2.274 (-3*pi/4) | 2.800 | 5.800 | goal | 1 | 1332 | 0.491 | 17.952 | 82.848 |
| indoor_2 | 9 | 3.200 | 8.200 | 1.642 (pi/2) | 7.000 | 2.600 | wall | 0 | 258 | 6.721 | 3.094 | 19.212 |
| indoor_2 | 10 | 5.000 | 6.200 | -0.508 (-0.162*pi) | 9.200 | 0.800 | goal | 1 | 2045 | 0.492 | 29.685 | 145.920 |
