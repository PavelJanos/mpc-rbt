# Úkol 1 - Význam parametrů mapy

Níže je shrnutí formátu mapových souborů v adresáři `maps` na základě reverzního inženýrství existujících příkladů.

## Struktura souboru mapy

Mapa je textový soubor, kde každá řádka obsahuje čísla oddělená čárkami:

1. `goal_x, goal_y`  
   Cílová pozice robota.

2. `min_x, min_y, max_x, max_y`  
   Hranice celé arény (obdélník mapy).

3. Následují řádky stěn ve formátu `x1, y1, x2, y2`  
   Každý řádek je jedna úsečka (zeď/překážka).

4. Volitelný oddělovač `inf`  
   Označuje konec definice stěn a začátek definice GNSS-denied oblasti.

5. Po `inf` následují polygon(y) GNSS-denied oblasti:  
   `x1, y1, x2, y2, x3, y3, ...`  
   Jeden řádek = jeden polygon (musí mít sudý počet hodnot, alespoň 3 vrcholy).

## Co znamenají parametry

- `goal_x, goal_y`: bod, kam má robot dojet.
- `min_x, min_y, max_x, max_y`: limitní obdélník, mimo který je robot považován za mimo arénu.
- `x1, y1, x2, y2` u stěn: krajní body úsečky překážky.
- `inf`: přepínač sekce na GNSS-denied zóny.
- Polygon po `inf`: oblast, kde GNSS nevrací validní data (simulace indoor prostředí).

## Praktická poznámka pro vlastní mapu

Pokud chcete simulovat vnitřní prostředí, je potřeba přidat GNSS-denied oblast:

1. Definujte stěny mapy.
2. Vložte řádek `inf`.
3. Přidejte alespoň jeden polygon GNSS-denied oblasti.

Bez této oblasti bude GNSS dostupné prakticky všude (spíše outdoor scénář).

## Jednoduchá šablona

```txt
goal_x,goal_y
min_x,min_y,max_x,max_y
x1,y1,x2,y2
x1,y1,x2,y2
...
inf
gx1,gy1,gx2,gy2,gx3,gy3,gx4,gy4
```
