# Task3 - Task3: Srovnani metod rizeni trasy

Testovano na mape `indoor_1` se startem `(2, 8.5)` a MoCap polohou.

- Pocet pokusu na metodu: 20
- Maximalni delka behu: 550 kroku
- Metody: `waypoint_p`, `pure_pursuit`, `cross_track_pd`, `stanley`

## Vysledky

| Metoda | Uspesnost [%] | Prum. kroky (jen uspech) | RMSE od trasy [m] | Kolize [pocet] | Mimo mapu [pocet] |
|---|---:|---:|---:|---:|---:|
| waypoint_p | 100.0 | 364.6 | 0.021 | 0 | 0 |
| pure_pursuit | 100.0 | 437.4 | 0.071 | 0 | 0 |
| cross_track_pd | 95.0 | 434.0 | 0.282 | 1 | 0 |
| stanley | 75.0 | 488.2 | 0.358 | 5 | 0 |

## Diskuze parametru metody

- `waypoint_p`: hlavni parametry jsou `K_heading`, `v_nom`, `wp_tol`. Vyssi `K_heading` zrychli nataceni, ale muze zpusobit kmitani.
- `pure_pursuit`: hlavni parametry jsou `lookahead_dist`, `v_nom`, `k_curve_slow`. Mensi lookahead zvysuje presnost, ale zhorsuje stabilitu.
- `cross_track_pd`: `K_cte`, `K_heading`, `K_d` urcuji kompromis mezi rychlou korekci a prekmitanim.
- `stanley`: `k_stanley`, `K_w`, `v_soft` ovlivnuji citlivost na pricnou chybu. Vyssi `k_stanley` drzi trasu tesneji, ale byva mene hladky.

## Vystupy

- `task3_motion_methods_summary.csv`
- `task3_motion_methods_comparison.png`
- `task3_motion_methods_results.mat`
