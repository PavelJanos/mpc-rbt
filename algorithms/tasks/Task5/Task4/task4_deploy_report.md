# Task5/Task4 - Nasazeni EKF bez znalosti pocatecni polohy

## Inicializace viry
- `mu_0` prevzato z GNSS inicializace (Task5/Task1): [2.0010, 1.9854, 0.0000]
- `Sigma_0` pouziva GNSS kovarianci pro `x,y` a vysokou varianci pro orientaci: `Sigma_{theta,0}=2.000`

## Naladene parametry filtru
- Base `R` (Task5/Task3): diag([0.00285, 0.00425, 0.00475])
- Coarse faze: pevna mrizka 7x7 (`R_scale` v [0.40, 1.60], `Q_scale` v [0.60, 1.60])
- Fine faze: mrizka 9x9 kolem coarse top (`R_center=0.400`, `Q_center=1.267`)
- Best scales: `R*=0.20 x R_base`, `Q*=1.18 x Q_base`
- Final `R` diag: [0.000570, 0.000850, 0.000950]
- Final `Q`:

```text
0.294074  0.018603
0.018603  0.325740
```

## Vysledky (best run)
- Dosazen cil: 1
- Pocet kroku: 547
- RMSE GNSS (XY): 0.7036 m
- RMSE EKF (XY): 0.1338 m
- Zlepseni EKF vuci GNSS: 81.0 %
- Mean |e_theta|: 0.1009 rad
- Smoothness (mean step-to-step EKF XY change): 0.0630 m/step

## Pozorovani
- Odhad polohy konverguje i bez zname pocatecni orientace.
- Orientace je uspesne odhadovana, i kdyz neni primo merena GNSS (stabilizace |e_theta|).
- Vyladeni R/Q zlepsilo kompromis mezi rychlou korekci a hladkym odhadem.

## Vystupy
- `task4_deploy_success_run.png`
- `task4_deploy_tuning_grid.png`
- `task4_deploy_tuning_table.csv`
- `task4_deploy_report.md`
