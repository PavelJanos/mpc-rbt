# Týden 2 - Nejistota

Cílem tohoto úkolu je seznámit se se simulátorem a prozkoumat nejistoty v senzorech a pohybu.

## Úkol 1 – Simulátor

Stáhněte repozitář obsahující simulátor z GitHubu [https://github.com/Robotics-BUT/MPC-MAP-Student](https://github.com/Robotics-BUT/MPC-MAP-Student) a seznamte se s ním (viz oddíl [Simulator](https://robotics-but.github.io/MPC-MAP-Student/resources/simulator/text.html)).
- Prozkoumejte datové struktury `private_vars`, `read_only_vars` a `public_vars`.
- Seznamte se s pořadím operací v nekonečné simulační smyčce v `main.m` a `algorithms/student_workspace.m`.
- Načtěte různé mapy přes `algorithms/setup.m` a nastavte různé startovní pozice.
- Vyzkoušejte různé pohybové příkazy v `algorithms/motion_control/plan_motion.m` a pozorujte chování robota.

V tomto kroku není vyžadován žádný výstup.

## Úkol 2 – Nejistota senzorů

Robot je vybaven osmikanálovým LiDARem a GNSS přijímačem. Určete směrodatnou odchylku (*std*) `sigma` pro data z obou senzorů tím, že umístíte robota do statické polohy (nulové rychlosti) v vhodných mapách a sesbíráte data po dobu nejméně 100 simulačních cyklů. Diskutujte, zda je *std* konzistentní napříč jednotlivými kanály LiDARu a oběma osami GNSS. Nakreslete histogramy měření.

## Úkol 3 – Kovarianční matice

Použijte měření z předchozího kroku a vestavěnou MATLAB funkci `cov` k vytvoření kovarianční matice pro oba senzory. Ověřte, že výsledná matice má rozměr 8×8 pro LiDAR a 2×2 pro GNSS. Ujistěte se, že hodnoty na hlavní diagonále jsou rovné `sigma^2`, tj. `variance=std^2`.

## Úkol 4 – Normální rozdělení

Vytvořte funkci `norm_pdf` pro sestavení funkce hustoty pravděpodobnosti (*pdf*) normálního rozdělení. Funkce by měla přijímat tři argumenty: `x` (hodnoty, pro které se *pdf* vyhodnocuje), `mu` (střední hodnota) a `sigma` (směrodatná odchylka). Využijte tuto funkci spolu s hodnotami `sigma` z úkolu 2 (např. pro první kanál LiDARu a osu X GNSS) k vygenerování dvou *pdf* znázorňujících charakter šumu senzorů robota a nakreslete je v jednom obrázku (použijte `mu=0` v obou případech).

## Úkol 5 – Nejistota pohybu

Nejistota existuje nejen v měřeních (senzorových datech), ale i v pohybu. Načtěte mapu `indoor_1` a pokuste se řídit robota k cílové pozici bez použití senzorů. K tomu použijte v `plan_motion.m` vhodnou posloupnost pohybových příkazů. Uložte snímek obrazovky úspěšného běhu a diskutujte o možných zdrojích nejistoty v pohybu robota.

## Odevzdání

Pro implementaci úkolů používejte pouze adresář `algorithms`; nemodifikujte zbytek simulátoru. Řešení musí fungovat bez chyb ve čerstvé instanci simulátoru a musí generovat grafické výstupy uvedené v protokolu.

Vytvořte jeden A4 protokol podle poskytnuté šablony, který stručně popisuje vaše řešení, s několika větami pro každý úkol a obrázkem, pokud je to vhodné.

Pošlete protokol a archiv `zip` obsahující adresář `algorithms` na e-mail učitele **do středy do 23:59 příštího týdne**.

Pro ty, kdo používají Git pro správu verzí, můžete místo souboru `zip` poslat odkaz na veřejné GitHub úložiště. Úložiště musí obsahovat simulátor s adresářem `algorithms` s vaším řešením. Označte prosím finální verzi tagem `week_2` pro snadnou identifikaci.