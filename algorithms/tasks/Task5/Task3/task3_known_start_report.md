# Task5/Task3 - Ladeni filtru se znamou pocatecni polohou (separate R)

- Start: `[2,2,pi/2]`
- Pocatecni vira: `mu=[2,2,pi/2]`, `Sigma=zeros(3,3)`
- Q: prevzata z Task5/Task1 (`task1_gnss_init_data.mat`), fallback `diag([0.25,0.25])`
- Opakovani na kandidata: 4
- Tuning: zvlast pro `R_xx`, `R_yy`, `R_tt` (coarse + fine)

## Nejlepsi nastaveni R
- `R = diag([0.002850, 0.004250, 0.004750])`
- Final run reached goal: 1
- Final run steps: 607
- RMSE GNSS (XY): 0.6963 m
- RMSE EKF (XY): 0.2075 m
- Improvement EKF vs GNSS: 70.2 %
- Mean |e_theta|: 0.0986 rad

## Vystupy
- `task3_known_start_ekf_result.png`
- `task3_R_tuning_progress.png`
- `task3_R_tuning_table.csv`
- `task3_ekf_tuning_results.csv`
- `task3_known_start_report.md`
