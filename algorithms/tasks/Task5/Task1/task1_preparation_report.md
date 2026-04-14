# Task5/Task1 - Příprava (outdoor_1)

## Trajektorie
- Mapa: `outdoor_1`
- Start: `[2,2,pi/2]`
- Cíl: `[16,2]`
- Trajektorie je ručně definována v `plan_path.m` (obsahuje křivkové segmenty).

## Inicializace GNSS (statické vzorkování v bodě startu)
- Počet platných vzorků: 450
- Odhad středu `mu = [2.0010, 1.9854]`
- Odhad směrodatných odchylek `std = [0.4985, 0.5247]`
- Kovarianční matice:

```text
0.248514  0.015721
0.015721  0.275273
```

Tyto hodnoty (`mu`, `Sigma`) lze přímo použít jako inicializační odhad GNSS pro další EKF úkoly.
