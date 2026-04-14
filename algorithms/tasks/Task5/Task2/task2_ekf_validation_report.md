# Task5/Task2 - EKF validace (predikce + korekce)

- Mapa: `outdoor_1`
- Start: `[2,2,pi/2]`
- Pocet kroku: 452

## Souhrn metrik
- RMSE GNSS (XY): 0.7094 m
- RMSE EKF (XY): 0.2374 m
- Zlepseni EKF vuci GNSS: 66.5 %
- Mean |e_theta|: 0.1083 rad
- Max  |e_theta|: 0.4261 rad

## Poznamka
- V tomto testu je rideni vedeno po referencni trajektorii a EKF je validovan proti true poloze.
