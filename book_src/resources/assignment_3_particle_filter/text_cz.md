# Týden 4 - Partiční filtr

Cílem tohoto úkolu je implementovat fungující lokalizační algoritmus založený na partičním filtru. Vaše odevzdané řešení by mělo být schopné ukázat konvergenci částic kolem robota.

## Úkol 1 – Predikce

Implementujte predikční funkci `predict_pose`, která vezme jako argumenty pozici částice a řídicí vstup a vrátí novou pozici. Aplikujte pravděpodobnostní model pohybu pro zvýšení variance částic.

## Úkol 2 – Korekce

Implementujte měřicí funkci `compute_lidar_measurement`, která jako argumenty vezme mapu, pozici částice a orientace lidarů a vrátí vektor změřených vzdáleností **bez** jakéhokoli šumu.

Můžete využít funkci simulátoru `ray_cast`, která se používá takto:

`intersections = ray_cast(ray_origin, walls, direction)`

Prvním argumentem je pozice počátečního bodu paprsku, druhým argumentem je popis stěn uložený v `read_only_vars.map.walls` a posledním argumentem je směr paprsku v radiánech. Funkce vrací všechny průsečíky daného paprsku se stěnami.

**Tip:** Můžete použít MoCap polohu a měření lidarů k ověření, že vaše funkce vrací správná data.

Poté implementujte funkci vážení `weight_particles`. Můžete použít libovolnou metrickou funkci, která splní požadavky.

## Úkol 3 – Převýběr

Implementujte funkci resamplingu `resample_particles`, která přijme sadu částic a jejich přidružené váhy jako argumenty a vrátí novou sadu převzorkovaných částic. Použijte libovolný algoritmus dle vašeho výběru.

## Úkol 4 – Lokalizace

Inicializujte sadu částic na náhodných pozicích v rámci libovolné vnitřní mapy. Aktualizujte partiční filtr v každé iteraci (proveďte predikci, korekci a resampling). Upravte dříve implementované funkce tak, aby se částice konvergovaly k pravé poloze robota. Měli byste být schopni prezentovat snímek s klastrem částic shromážděných kolem agenta. V některých případech možná budete muset robota pohnout, aby lokalizace fungovala.
 
**Tip:** Začněte v mapě s výraznými rysy, tj. neobsahující podobné chodby apod. Pro zjednodušení si můžete vytvořit vlastní mapu.

Diskutujte nejdůležitější parametry vašeho řešení a zdůvodněte výběr algoritmů. Jaký byl hlavní problém, který jste museli překonat?

## Odevzdání

Pro implementaci úkolů používejte pouze adresář `algorithms`; nemodifikujte zbytek simulátoru. Řešení musí fungovat bez chyb ve čerstvé instanci simulátoru a musí generovat grafické výstupy uvedené v protokolu.

Vytvořte jeden A4 protokol podle poskytnuté šablony, který stručně popisuje vaše řešení, s několika větami pro každý úkol a obrázkem, pokud je to vhodné.

Pošlete protokol a archiv `zip` obsahující adresář `algorithms` na e-mail učitele **do středy do 23:59 příštího týdne**.

Pro ty, kdo používají Git pro správu verzí, můžete místo souboru `zip` poslat odkaz na veřejné GitHub úložiště. Úložiště musí obsahovat simulátor s adresářem `algorithms` s vaším řešením. Označte prosím finální verzi tagem `week_4` pro snadnou identifikaci.