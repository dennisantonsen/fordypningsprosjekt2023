# Fordypningsprosjekt2023

TODO: English translation?

Industrianlegg har gjerne tusenvis av ventiler, sensorer og andre elementer som bør inspiseres visuelt med jevne mellomrom, og autonome roboter kan brukes til å automatisere denne prosessen. Det krever at roboten er i stand til å finne en tilgjengelig posisjon som gir god sikt til hvert aktuelle element. Dersom anlegget har en god digital tvilling kan planleggingen i mange tilfeller gjøres på forhånd ut fra 3D-modellen av anlegget. I en masteroppgave våren 2023, samt tidligere arbeider, har det blitt implementert en optimaliseringsalgoritme for inspeksjon med 3D-modellen av anlegget Huldra som testområde. En svakhet ved de eksisterende algoritmene er at de har for lang kjøretid ved bruk i store anlegg, og et forsøk på å lage en mer effektiv søkealgoritme førte til at mindre optimale lokale minimum ble funnet i mange tilfeller. Algoritmene er også noe begrenset med hensyn på hvilke lovlige posisjoner som finnes, og robotens fleksibilitet til å flytte kameraet horisontalt og vertikalt i forhold til robotens base.

I denne oppgaven settes fokus på å øke fleksibiliteten for kameraplassering i forhold til roboten, og på å utarbeide en bedre søkealgoritme for robotposisjonen.

Prosjektet kan oppsummeres i de følgende punktene:
- Skaffe oversikt over verktøykjeden og algoritmene etablert i tidligere arbeider
- Parametrisere kameraposisjon i forhold til robotens base som funksjon av lengde på arm og mulige vinkler.
- Etablere søkealgoritme som finner optimal plassering av robot og kamera.
- Vurdere og teste tiltak for å unngå at algoritmen finner suboptimale lokale minium.
- Hvis mulig, forberede test av algoritmen ved tilgjengelig fysisk anlegg med fysisk robot.
- Gi anbefalinger for det videre arbeidet

Tidligere arbeid:
- [erlend master 2021](https://github.com/erlendb/ntnu-masteroppgave)
- [aashild ntnu oppgave 2022](https://github.com/aashilbr/aashild-ntnu-oppgave-2022/tree/produce_json)  
  Branch `produce_json` er den som inneholder siste status.


## Components

  - [godot](./godot/) - 3d game engine 
  - [simulator](./simulator/) - Robot simulator 