# Task4/Task4 - Lokalizace partičním filtrem

## Nastaveni
- Mapa: `indoor_1`
- Start: `(2, 8.5, pi/2)`
- Pocet castic: 700
- Predikce: probabilisticky motion model (`predict_pose`)
- Korekce: LiDAR model + gaussovske vahovani (`weight_particles`)
- Resampling: `systematic`

## Vysledky
- Nejlepsi detekovana kompaktnost klastru (median vzdalenosti): 0.033 m
- Prumerna XY chyba odhadu: 0.053 m
- Minimalni XY chyba odhadu: 0.016 m
- Maximalni XY chyba odhadu: 0.289 m

## Diskuze nejdulezitejsich parametru
- Pocet castic (`N`): vyssi N zvysuje robustnost a presnost, ale roste vypocetni narocnost.
- `sigma_lidar` ve vahovani: mala hodnota vede k ostremu vahovani a riziku degenerace, velka hodnota zhorsuje rozliseni mezi casticemi.
- Sila sumu v `predict_pose`: prilis maly sum omezuje prohledani stavu, prilis velky sum zhorsuje stabilitu odhadu.
- Volba resamplingu: `systematic` poskytuje nizkou varianci a dobry kompromis rychlost/kvalita.

## Hlavni problem a jeho reseni
- Hlavni problem byla degenerace vah pri vetsim nesouladu mereni. Resenim bylo stabilni gaussovske vahovani, validace kanalu LiDARu a systematic resampling.

## Vystupy
- `task4_pf_cluster_snapshot.png`
- `task4_pf_metrics.png`
- `task4_task4_report.md`
