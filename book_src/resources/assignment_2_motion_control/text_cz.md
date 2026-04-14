# Týden 3 - Řízení pohybu

Tento týden implementujte jednoduchý algoritmus sledování dráhy pomocí lokalizačních údajů z motion capture. Vaše odevzdané řešení by mělo být schopné dovést robota k cíli podél definované trasy.

## Úkol 1 – Vytvoření map

Prozkoumejte příklady v adresáři `maps` a pomocí reverzního inženýrství se pokuste zjistit význam jednotlivých parametrů. Pokud si přejete vytvořit vlastní mapu pro testování algoritmů, musíte definovat oblast bez GNSS signálu, abyste simulovali vnitřní prostředí, kde motion capture funguje.

V tomto kroku není vyžadován žádný výstup.

## Úkol 2 – Vytvoření trasy

Definujte trasu v mapě `indoor_1`, která vede z bodu `(2, 8.5)` do cílové pozice. Trasa nesmí sestávat pouze z přímek, zařaďte také křivky (např. kruhový oblouk nebo sinusovku). Udržujte bezpečnou vzdálenost mezi segmenty trasy a stěnami.

Trasa je definována jako posloupnost bodů (x, y) (waypointy). Upozorňujeme, že waypointy uložené v proměnné `public_vars.path` jsou simulátorem vizualizovány. Doporučuje se tuto proměnnou využít.

## Úkol 3 – Řízení pohybu

Nastavte počáteční pozici robota tak, aby odpovídala začátku definované trasy – `(2, 8.5)`. Zvolte libovolný algoritmus sledování trasy a implementujte ho tak, aby robot sledoval trasu definovanou v předchozím úkolu. Využijte proměnnou `read_only_vars.mocap_pose` pro získání téměř pravé polohy robota. Diskutujte, jak parametry zvoleného metody ovlivňují kvalitu řízení pohybu.

Vaše řešení uložte do funkce `plan_motion`. Výchozí struktura kódu je připravena pro algoritmy založené na cílovém bodu; můžete však upravit funkci a použít např. teorii chyby příčné odchylky. Volba záleží na vašich preferencích; můžete implementovat i více algoritmů a porovnat je.

Mějte na paměti, že data MoCap nebudou k dispozici pro závěrečný projekt!

## Odevzdání

Pro implementaci úkolů používejte pouze adresář `algorithms`; nemodifikujte zbytek simulátoru. Řešení musí fungovat bez chyb ve čerstvé instanci simulátoru a musí generovat grafické výstupy uvedené v protokolu.

Vytvořte jeden A4 protokol podle poskytnuté šablony, který stručně popisuje vaše řešení, s několika větami pro každý úkol a obrázkem, pokud je to vhodné.

Pošlete protokol a archiv `zip` obsahující adresář `algorithms` na e-mail učitele **do středy do 23:59 příštího týdne**.

Pro ty, kdo používají Git pro správu verzí, můžete místo souboru `zip` poslat odkaz na veřejné GitHub úložiště. Úložiště musí obsahovat simulátor s adresářem `algorithms` s vaším řešením. Označte prosím finální verzi tagem `week_3` pro snadnou identifikaci.