# TODO - Semestralni projekt

## Hlavni cil

Vyvinout robustni reseni pro simulator, ktere:
- funguje bez MoCap,
- funguje na vsech vzorovych mapach,
- zvladne ruzne startovni a cilove pozice,
- nekoliduje se stenami,
- zustava v hranicich mapy,
- a dojede do cile v co nejmensim poctu iteraci.

## Navrzena architektura

Robustni reseni bude postavene na teto pipeline:
- lokalizace bez MoCap,
- bezpecne planovani trasy,
- opatrne sledovani trasy,
- bezpecnostni vrstva proti kolizi,
- systematicke testovani na vsech mapach.

## Krok 1 - Lokalizace

- Pouzit `PF` z LiDARu jako hlavni indoor lokalizaci.
- Pouzit `EKF` z GNSS jako hlavni outdoor lokalizaci.
- Pri soucasne dostupnosti obou odhadu pouzit robustni fuzi EKF+PF s gatingem.
- Na zacatku jizdy pridat kratkou `warm-up` fazi, dokud se odhad nestabilizuje.
- Nepouzivat `MoCap` v produkcnim reseni.

## Krok 2 - Rozhodovani podle senzoru

- Kdyz je `GNSS` validni, preferovat EKF odhad.
- Kdyz je `GNSS` nevalidni a `LiDAR` validni, preferovat PF odhad.
- Kdyz jsou validni oba, pouzit fuzi nebo gating.
- Kdyz neni validni nic, prejit do bezpecneho modu (`stop`).

## Krok 3 - Planovani trasy

- Pouzit `A*` jako default planner.
- Zachovat clearance od prekazek pomoci inflace mapy.
- Planner musi fungovat pro libovolnou startovni a cilovou pozici, pokud reseni existuje.
- Vyhnout se specialnim mapovym hackum, pokud nejsou nutne.

## Krok 4 - Vyhlazeni trasy

- Pouzit `chaikin` jako default smoothing.
- Vyhlazeni aplikovat jen pokud neporusi clearance.
- Pokud smoothing zhorsuje bezpecnost, vratit puvodni trasu.
- Ostatni smoothing metody nechat jen pro benchmark a porovnani.

## Krok 5 - Rizeni

- Pouzit `pure pursuit` jako default sledovani trasy.
- Omezit rychlost pri velke uhlove chybe.
- Na zacatku jizdy nejdriv srovnat robota s prvnim segmentem trasy.
- Nejezdit agresivne v momente, kdy lokalizace jeste neni stabilni.

## Krok 6 - Bezpecnostni vrstva

- Pokud LiDAR vidi prekazku prilis blizko pred robotem, nezrychlovat slepe po trase.
- Pridat jednoduchy `emergency stop` nebo vyhybaci reakci.
- Pri zjevne spatnem odhadu polohy robot nesmi pokracovat plnou rychlosti.
- Pri velke nekonzistenci mezi EKF a PF prejit do konzervativniho rezimu.

## Krok 7 - Startovni overconfidence

- Nepredpokladat na zacatku malou nejistotu, pokud neni opravdu podlozena daty.
- Na startu drzet vetsi nejistotu u polohy a hlavne u orientace.
- Pozvolna "dovolovat" rizeni az po stabilizaci odhadu.
- Osetrit pripad, kdy filtr na zacatku konverguje na spatnou hypotezu.

## Krok 8 - Benchmark a testovani

Vytvorit projektove skripty do teto slozky:
- `project_test_all_maps.m`
- `project_random_benchmark.m`
- `project_debug_indoor.m`
- `project_debug_outdoor.m`
- `project_compare_localization_modes.m`
- `project_compare_controllers.m`

Tyto skripty by mely:
- spoustet opakovane behy na vice mapach,
- menit start a cil,
- merit uspesnost,
- merit pocet kroku,
- zaznamenavat kolize,
- a ukladat grafy/tabulky pro dalsi ladeni.

## Krok 9 - Vlastni testovaci mapy

- Pridat vlastni sadu map do podslozky, napr. `maps_custom/`.
- Vytvorit mapy s:
  - uzkymi chodbami,
  - symetrickymi useky,
  - slepymi ulickami,
  - dlouhymi rovinkami,
  - kombinaci indoor/outdoor podminek.

## Krok 10 - Finalni produkcni rezim

- Produkcni logika musi zustat v:
  - `student_workspace.m`
  - `plan_path.m`
  - `plan_motion.m`
- Testovaci skripty musi zustat v `algorithms/tasks/Project`.
- Projekt nesmi zaviset na toolboxech.
- Projekt musi byt spustitelny v MATLAB R2023b.

## Doporucene poradi prace

1. Stabilizovat lokalizaci bez MoCap.
2. Dokoncit robustni fuzi EKF+PF s gatingem.
3. Zafixovat `A*` + clearance.
4. Zafixovat `chaikin` smoothing.
5. Zafixovat `pure pursuit` + pridat bezpecnostni vrstvu do rizeni.
6. Vytvorit benchmark skripty pro vsechny mapy.
7. Ladit podle vysledku benchmarku, ne podle jednoho behu.

## Poznamka

Robustni rizeni nevznika z agresivniho controlleru, ale z toho, ze:
- robot jede jen kdyz ma rozumne spolehlivy odhad,
- planner drzi bezpecny odstup,
- smoothing nezhorsuje clearance,
- a testy se delaji systematicky na vice mapach a vice startech.
