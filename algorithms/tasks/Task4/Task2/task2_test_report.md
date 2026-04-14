# Task4/Task2 - Testovaci report

## compute_lidar_measurement
- Prumerna odchylka |model-simulator|: 0.0417 m
- Maximalni odchylka |model-simulator|: 0.1081 m
- Poznamka: simulatorova funkce muze obsahovat mereni se sumem.

## weight_particles
- Test poradi vah (idealni > maly sum > velky sum): PASS
- Test normalizace sum(weights)=1: PASS
- Robustnost pro NaN/Inf v mereni: PASS
- Vahy (test poradi): [0.5414, 0.4566, 0.001947]
- Pri sigma=0.00: max(w)=0.001667, ESS=600.0
- Pri sigma=0.35: max(w)=0.2444, ESS=8.5

## Soubory
- task2_measurement_weighting_test.png
- task2_test_report.md
