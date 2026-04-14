# Týden 5 - Kalmanův filtr a EKF

Cílem tohoto úkolu je implementovat lokalizační algoritmus založený na rozšířeném Kalmanově filtru a datech GNSS.

## Úkol 1 – Příprava

Načtěte mapu `outdoor_1` a ručně navrhněte trajektorii mezi počáteční polohou `[2,2,π/2]` a cílovou pozicí `[16,2]`.

Prozkoumejte nejistotu měření GNSS; tj. implementujte inicializační postup, který určí počáteční pozici (*střed*) a kovarianční matici dat senzoru GNSS. Tyto údaje budete později potřebovat v dalších úkolech.

## Úkol 2 – Implementace EKF

Jelikož používáte robot s diferenciálním pojezdem, je ve stavové a řídicí přechodové funkci nelinearita (nelineární funkce *g(x)*); proto musíte v predikční fázi použít EKF. Implementujte funkci `ekf_predict` pro daný typ pohonu.

Na druhou stranu je KF vhodný pro fázi korekce, protože mezi stavem a měřením je lineární vztah. Implementujte funkci `kf_correct`.

## Úkol 3 – Ladění filtru s známou počáteční polohou

Nastavte počáteční polohu na `x=[2,2,π/2]`. Použijte tuto polohu a Σ s nulami (tj. vysoká jistota) jako počáteční víru pro algoritmus EKF a navigujte robota po vaší trajektorii k cílové poloze. Jako jedinou informaci pro řízení může být použita pouze EKF–odhadnutá poloha. Kovarianční matici měření `Q` sestavíte z dříve známé kovariance GNSS; pro šum procesu použijte počáteční odhad variančních hodnot 0.01 pro všechny proměnné a dolaďte matici `R`, abyste dosáhli optimálního chování. Zachyťte výsledek.

## Úkol 4 – Nasazení algoritmu

V praxi není počáteční poloha pro algoritmus známa. Využijte inicializační postup z úkolu 1 pro určení počáteční víry pro EKF. Všimněte si, že orientaci robota nemůžete měřit, proto je třeba v počáteční víře zadat pro tuto proměnnou vysokou varianci.

Pozorujte, jak odhadnutá poloha konverguje k reálným hodnotám, a nakonec upravte parametry filtru, abyste dosáhli optimálního chování („hladký“ odhad). Upozorněte, že orientace je úspěšně odhadnuta i když není měřena přímo. Zachyťte úspěšnou jízdu k cíli.

## Odevzdání

Pro implementaci úkolů používejte pouze adresář `algorithms`; nemodifikujte zbytek simulátoru. Řešení musí fungovat bez chyb ve čerstvé instanci simulátoru a musí generovat grafické výstupy uvedené v protokolu.

Vytvořte jeden A4 protokol podle poskytnuté šablony, který stručně popisuje vaše řešení, s několika větami pro každý úkol a obrázkem, pokud je to vhodné.

Pošlete protokol a archiv `zip` obsahující adresář `algorithms` na e-mail učitele **do středy do 23:59 příštího týdne**.

Pro ty, kdo používají Git pro správu verzí, můžete místo souboru `zip` poslat odkaz na veřejné GitHub úložiště. Úložiště musí obsahovat simulátor s adresářem `algorithms` s vaším řešením. Označte prosím finální verzi tagem `week_5` pro snadnou identifikaci.