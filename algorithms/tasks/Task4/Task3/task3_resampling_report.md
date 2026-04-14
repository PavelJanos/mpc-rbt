# Task4/Task3 - Porovnani resamplingu

- Pocet pokusu: 250
- Pocet castic: 1200

| Metoda | Mean XY error [m] | Std XY error [m] | Unique ratio | Runtime [ms] |
|---|---:|---:|---:|---:|
| multinomial | 0.0257 | 0.0135 | 0.054 | 2.0093 |
| systematic | 0.0058 | 0.0030 | 0.060 | 0.0323 |
| stratified | 0.0068 | 0.0035 | 0.058 | 0.0298 |
| residual | 0.0083 | 0.0044 | 0.057 | 0.1812 |

Doporuceni: pro tento projekt je vychozi volba `systematic` kvuli nizke varianci a dobre rychlosti.
