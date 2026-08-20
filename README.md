# Smart Charge System

Smart Charge System to symulacyjny system planowania tras dla floty dronów z uwzględnieniem ograniczeń energetycznych, stacji ładowania oraz kolizji pomiędzy agentami. Projekt pokazuje, w jaki sposób wiele dronów może autonomicznie realizować zadania w środowisku miejskim, wybierając trasy prowadzące przez punkty dostawy, lądowiska i - jeśli to konieczne - stacje ładowania.

Główną motywacją projektu jest rosnące znaczenie automatyzacji oraz wykorzystania dronów w logistyce, usługach, inspekcjach i przemyśle. Wraz ze wzrostem popularności takich rozwiązań kluczowe staje się nie tylko samo wyznaczenie trasy z punktu A do punktu B, ale także uwzględnienie praktycznych ograniczeń: pojemności baterii, dostępności ładowarek, bezpieczeństwa ruchu i koordynacji wielu jednostek w tej samej przestrzeni. Proponowany scenariusz zakłada wykorzystanie stacjonarnych punktów ładowania, które w przyszłości mogłyby znajdować się np. na dachach budynków.

## Najważniejsze funkcje

- planowanie tras dla 40 dronów w scenariuszu miejskim inspirowanym Gliwicami,
- reprezentacja miasta jako mapy siatkowej 72 x 56, gdzie jedno pole odpowiada obszarowi 250 m x 250 m,
- uwzględnienie przeszkód i uproszczonych stref niedostępnych dla lotu,
- model energii drona z kosztem ruchu, minimalnym zapasem końcowym i możliwością ładowania,
- obsługa stacji ładowania o ograniczonej pojemności,
- planowanie trasy w kolejności: start -> punkt dostawy -> lądowisko,
- unikanie konfliktów wierzchołkowych, konfliktów krawędziowych oraz przeciążenia ładowarek,
- wykorzystanie różnych pułapów lotu: 60 m, 80 m, 100 m i 120 m,
- wizualizacja tras na mapie oraz możliwość animacji ruchu dronów,
- test integracyjny sprawdzający, czy wszystkie drony kończą zadania bez konfliktów.

## Podstawy teoretyczne

Projekt nawiązuje do problemu MAPF, czyli Multi-Agent Path Finding. Jest to zagadnienie planowania tras wielu agentów poruszających się we wspólnym środowisku, w którym należy unikać wzajemnych kolizji.

W implementacji wykorzystano podejście inspirowane klasycznym algorytmem A*, opisanym przez Harta, Nilssona i Raphaela [2]. Dla pojedynczego drona używana jest heurystyka oparta na odległości Manhattan, która prowadzi trasę przez punkt dostawy, a następnie do lądowiska.

Do koordynacji wielu dronów zastosowano podejście CBS, czyli Conflict-Based Search. Sharon, Stern, Felner i Sturtevant [3] opisują CBS jako metodę rozwiązywania MAPF przez wykrywanie konfliktów pomiędzy agentami i dodawanie ograniczeń, które wymuszają ponowne planowanie wybranych tras.

Niskopoziomowe planowanie pojedynczej trasy korzysta z idei EPEA*, czyli Enhanced Partial Expansion A*. Goldenberg i współautorzy [1] pokazują, że częściowa ekspansja stanów pozwala ograniczać liczbę generowanych następców, co jest istotne w problemach o dużej przestrzeni stanów.

Wizualizacja wyników została przygotowana z użyciem biblioteki Matplotlib, opisanej przez Huntera [5] jako środowisko do tworzenia dwuwymiarowych wykresów naukowych.

## Scenariusz Gliwice

Domyślny scenariusz generuje symulację dla 40 dronów:

- 20 dronów startuje z lewej strony mapy,
- 20 dronów startuje z prawej strony mapy,
- każdy dron otrzymuje własny punkt dostawy,
- po wykonaniu zadania dron kieruje się do przypisanego lądowiska,
- na mapie znajduje się 12 stacji ładowania,
- każda stacja ma ograniczoną pojemność,
- trasy są walidowane pod kątem kolizji i poprawności energetycznej.

Przykładowy wynik dla aktualnego scenariusza:

```text
Planner: CBS + EPEA*
Liczba dronów: 40
Czas zakończenia wszystkich lotów: 155
Liczba kroków ładowania: 46
Liczba wykrytych konfliktów po walidacji: 0
