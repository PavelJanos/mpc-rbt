# Semestrální projekt

## Cíle projektu

Hlavním cílem semestrálního projektu je vyvinout použitelné řešení pro daný MATLAB simulátor. Váš kód by měl vést robota k jeho cíli v co nejmenším počtu iterací. Algoritmy by měly fungovat v různých typech map a obstarávat libovolné cílové a startovní pozice včetně orientace. Je nezbytné, aby robot vyhnul se kolizím s překážkami a zůstal v hranicích mapy.

Úspěšné dokončení projektu vyžaduje schopnost úkol splnit ve všech vzorových mapách obsažených ve složce `maps`. I když byly v přednáškách a úkolech pokryty vhodné metody, nemusíte použít všechny prezentované postupy. Je dovoleno upravovat soubory v adresáři `algorithms` a přidávat vlastní funkce; úprava skriptu `main.m` je však striktně zakázána. **Pozor, krok 7 hlavní smyčky simulátoru bude deaktivován, čímž data MoCap nebudou dostupná.**

## Simulátor

Simulátor je dostupný v repozitáři <https://github.com/Robotics-BUT/MPC-MAP-Student>. Nemodifikujte žádné funkce, na které simulátor spoléhá; vaše řešení bude hodnoceno pomocí funkcí zveřejněných v repozitáři.

**Váš kód musí být spustitelný bez závislostí na jakémkoli MATLAB toolboxu.**

Řešení bude hodnoceno v MATLAB R2023b. **Projekt nesmí při spuštění obsahovat chyby.**

*Tip:*
Naklonujte „čistou“ kopii simulátoru z repozitáře a použijte instalaci MATLABu bez toolboxů, aby váš projekt nenarazil na problémy.

## Hodnocení

Projekt bude testován vyučujícími pomocí různých map a náhodně vybraných souřadnic pro startovní a cílové pozice (včetně orientace). Úspěšnost dosažení cíle a potřebný počet iterací budou statisticky vyhodnoceny opakovaným spuštěním kódu.

## Odevzdání

Pošlete archiv `zip` obsahující adresář `algorithm` na e-mailovou adresu <tomas.lazna@ceitec.vutbr.cz> do středy 23:59 9. týdne semestru (9. dubna 2025).

Pro ty, kdo používají Git pro správu verzí, můžete místo souboru `zip` poslat odkaz na veřejné GitHub úložiště. Úložiště musí obsahovat simulátor s adresářem `algorithms` s vaším řešením. Označte prosím finální verzi tagem `project` pro snadnou identifikaci.

## Bodovací systém
- Úspěšnost dosažení cíle: až 15 bodů
- Potřebný počet iterací: až 15 bodů
- Technická kvalita: až 20 bodů

Pro úspěšné absolvování kurzu musíte získat minimálně 20 bodů ze semestrálního projektu.