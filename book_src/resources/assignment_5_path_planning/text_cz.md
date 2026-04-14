# Týden 6 - Plánování trasy

Cílem tohoto úkolu je implementovat fungující algoritmus plánování trasy. Vaše odevzdané řešení by mělo být schopné naplánovat trasu mezi libovolnou startovní a cílovou pozicí (pokud řešení existuje).

## Úkol 1

Zvolte a implementujte algoritmus, který najde trasu mezi startovní pozicí robota a cílovou pozicí v mapě. Trasa se nesmí srážet s žádnou stěnou. Použijte obsazenostní mřížku uloženou v proměnné `read_only_vars.discrete_map.map`.

## Úkol 2

Upravte plánovací algoritmus tak, aby zachoval odstup od překážek. Můžete použít libovolnou metodu. Odstup musí být alespoň 0,2 m.

## Úkol 3

Aplikujte na vygenerované trasy vyhlazovací algoritmus. Použijte iterativní algoritmus uvedený v přednášce nebo najděte jiný použitelý postup. Diskutujte vliv parametrů algoritmu.

## Odevzdání

Pro implementaci úkolů používejte pouze adresář `algorithms`; nemodifikujte zbytek simulátoru. Řešení musí fungovat bez chyb ve čerstvé instanci simulátoru a musí generovat grafické výstupy uvedené v protokolu.

Vytvořte jeden A4 protokol podle poskytnuté šablony, který stručně popisuje vaše řešení, s několika větami pro každý úkol a obrázkem, pokud je to vhodné.

Pošlete protokol a archiv `zip` obsahující adresář `algorithms` na e-mail učitele **do středy do 23:59 příštího týdne**.

Pro ty, kdo používají Git pro správu verzí, můžete místo souboru `zip` poslat odkaz na veřejné GitHub úložiště. Úložiště musí obsahovat simulátor s adresářem `algorithms` s vaším řešením. Označte prosím finální verzi tagem `week_6` pro snadnou identifikaci.