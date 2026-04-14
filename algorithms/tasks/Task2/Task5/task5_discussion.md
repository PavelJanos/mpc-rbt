# Task 5 - Diskuze nejistoty pohybu

Open-loop řízení (pevná posloupnost příkazů v `plan_motion.m`) může selhat, i když stejná sekvence jednou fungovala. Hlavní zdroje nejistoty:

1. Prokluz kol a proměnlivé tření povrchu.
2. Nepřesný model poloměru kol a vzdálenosti kol.
3. Rozdíl mezi požadovanou a skutečně vykonanou rychlostí kol.
4. Integrační/diskretizační chyba během mnoha kroků.
5. Nepřesná počáteční poloha (malá chyba se v open-loop hromadí).
6. Dynamika akčních členů a saturace při otáčení.

Důsledek: nejistota se po trase postupně kumuluje, zejména po více zatáčkách a delších přímých úsecích.
