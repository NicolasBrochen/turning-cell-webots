[English](README.md) · **Français**

# Cellule de tournage robotisée — Webots R2025a

Deux tours CNC face à face, à 1,6 m l'un de l'autre, servis par un **Universal Robots UR10e**. Le robot porte une
**double pince de type SCHUNK** (deux pinces parallèles sur un support coudé à 90°, chacune à 45° de l'axe de la bride)
montée derrière un **capteur d'effort 6 axes**. Les bruts arrivent dans une caisse,
et les pièces tournées repartent dans une seconde caisse. Quand la caisse de bruts est vide, un opérateur vient échanger
les caisses. Le côté avant, ouvert et collaboratif, est surveillé par deux scrutateurs laser (surveillance de vitesse
et de distance).

**▶ [Animation 3D interactive dans le navigateur](https://nicolasbrochen.github.io/turning-cell-webots/)** (GitHub Pages, sans installer Webots) · [vidéo de la séquence complète, accélérée ×4 (MP4)](docs/turning_cell.mp4)

[![Séquence complète : bruts, service des tours avec la double pince, tournage, pièces finies, échange des caisses](docs/turning_cell.gif)](docs/turning_cell.mp4)

| Service d'un tour avec la double pince | Échange des caisses (scrutateurs laser) |
|---|---|
| [![Service d'un tour](docs/gripper.jpg)](docs/lathe_service.mp4) | [![Échange des caisses](docs/operator.jpg)](docs/crate_exchange.mp4) |

## Visualiser sans Webots

- **Animation 3D interactive** : [nicolasbrochen.github.io/turning-cell-webots](https://nicolasbrochen.github.io/turning-cell-webots/).
  C'est un export HTML5 de Webots de 4 min 06 s de production (les deux tours, les échanges de pinces, l'échange des
  caisses) : on peut tourner autour, zoomer et avancer dans le temps. Fichiers dans [`site/`](site/), publiés depuis la
  branche `gh-pages` (`git subtree push --prefix site origin gh-pages`).
- **Vidéos** : [`docs/`](docs/).
- **Reconstruire** : `tools/build_site.sh` (animation) et `tools/build_media.sh` (vidéos, GIF, images). Sous WSL, ils
  utilisent automatiquement **Webots Windows** s'il est installé avec un Python Windows (rendu direct sur le GPU) ;
  `WEBOTS_MODE=linux` force le Webots Linux ([`tools/webots_env.sh`](tools/webots_env.sh)). Laisser la fenêtre Webots
  visible pendant l'enregistrement des vidéos.

## Lancer dans Webots

```
webots worlds/tournage.wbt
```

## Cycle

1. **Prise d'un brut** (pince 1) dans la caisse bleue : la pince descend à la verticale, les doigts se ferment et les
   capteurs de bout de doigt confirment la prise (force de serrage lue sur les moteurs de doigts, ~140 N par doigt).
2. **Service d'un tour en une seule visite**, grâce à la double pince :
   - la pince 2 entre par la porte ouverte, saisit la pièce tournée, l'automate ouvre le mandrin, le robot extrait la pièce ;
   - un demi-tour du poignet 3 présente la pince 1 devant le mandrin (le support coudé fait permuter les deux
     pinces) : le brut est poussé le long de l'axe de broche en
     **mode effort**, et le capteur d'effort du poignet arrête le mouvement quand la pièce touche la butée du mandrin
     (seuil 40 N) ;
   - l'automate serre le mandrin, la pince s'ouvre, le robot sort de la machine, revient au-dessus de la table et
     signale « dégagé » (la porte ne se ferme qu'une fois le bras hors de son chemin).
3. **Cycle de tournage** (automate) : la porte se ferme et se verrouille, la broche démarre, la tourelle approche et
   fait une passe avec arrosage. La surépaisseur disparaît au fur et à mesure de l'avance de l'outil. Puis la tourelle
   se retire, la broche s'arrête et la porte s'ouvre.
4. **Dépose de la pièce finie** (pince 2) dans la caisse verte : la pièce est posée en mode effort (contact avec le fond
   détecté à 20 N), puis relâchée.
5. Le robot alterne entre le tour A et le tour B, en servant toujours d'abord celui qui a terminé.
6. **Échange des caisses** : quand la caisse de bruts est vide, l'opérateur vient à la table. Les scrutateurs ralentissent
   d'abord le robot (25 % en champ d'avertissement), puis l'arrêtent (champ de protection). L'opérateur échange les
   caisses et repart, et le robot reprend automatiquement à pleine vitesse.

## Modèles

| Élément | Source |
|---|---|
| Robot | `UR10e` (bibliothèque officielle Webots, Universal Robots) sur socle acier |
| Double pince | Construite pour ce projet : capteur d'effort 6 axes, plaque d'adaptation alu, support coudé à 90° avec goussets, 2 pinces parallèles (corps type SCHUNK EGK) à ±45° de l'axe de la bride, doigts motorisés sur glissières, patins avec capteurs de contact, LED d'état. SCHUNK ne fournit pas de modèle Webots. Géométrie dans [`dual_gripper.py`](controllers/ur_tending/dual_gripper.py) |
| Tours CNC | Construits pour ce projet (aucun tour dans la bibliothèque Webots) : banc fonte, carénage, porte coulissante automatique vitrée, poupée, mandrin 3 mors (mors mobiles), banc incliné à 45°, tourelle 8 postes avec plaquette, arrosage, pupitre CNC, colonne lumineuse, convoyeur et bac à copeaux |
| Caisses | Construites pour ce projet : bacs plastique 400×600×180 (type KLT) avec nervures, poignées et 12 alvéoles |
| Pièces | Brut Ø50 × 90 mm → arbre étagé Ø40 / Ø30 (la zone serrée reste brute) |
| Opérateur | `Pedestrian` (bibliothèque officielle), marche animée par l'automate avec les tables de pas officielles |
| Atelier | Bardage, poteaux, fermes et luminaires, rack à barres, établi, armoire à outils, deux autres tours |

La géométrie statique est fusionnée par matériau en `IndexedFaceSet` (même méthode que la cellule de palettisation).

## Capteurs

| Capteur | Réalisation | Usage |
|---|---|---|
| **Capteurs d'effort de bout de doigt** (×4) | `TouchSensor` Webots (contact) sur chaque doigt + retour d'effort des moteurs de doigts (`LinearMotor.getForceFeedback`) | Contrôle de prise : les deux doigts en contact à ≥ 50 N, sinon « grip failed » et réouverture |
| **Capteur d'effort 6 axes au poignet** | Émulé par le contrôleur robot (raideur de contact 40 kN/m au-delà du point de contact) | Mouvements gardés en effort : mise en butée du brut dans le mandrin (40 N), dépose dans la caisse (20 N) |
| **Codeurs articulaires** | `PositionSensor` sur les 6 axes et les 4 doigts | Cinématique, suivi de la pièce transportée, course des doigts |
| **Scrutateurs laser** (×2) | Éventails de 31 faisceaux de distance à 0,18 m | Champ d'avertissement → 25 % de vitesse, champ de protection → arrêt de protection, reprise automatique |
| Interverrouillages porte / mandrin | États de l'automate | Pas de broche porte ouverte, pas d'entrée robot broche en rotation |

## Sécurités

- **Côté avant collaboratif** (ISO/TS 15066, surveillance de vitesse et de distance) : 2 scrutateurs laser, champ
  d'avertissement (orange, de 2,0 à 2,8 m) et champ de protection (rouge, de 1,4 à 2,0 m) marqués au sol.
- **Carter fixe** à l'arrière entre les deux machines (profilés alu + polycarbonate, même `GuardBay.proto` que la
  cellule de palettisation).
- **Portes des tours** interverrouillées : fermées et verrouillées pendant le tournage, ouvertes seulement broche arrêtée.
- **Arrêts d'urgence** sur la table et sur chaque pupitre CNC : défaut mémorisé, réarmement refusé tant que le bouton
  est enfoncé.
- **Colonnes lumineuses** sur chaque tour (vert = tournage, orange = attente robot, rouge = défaut).

## Collisions

Les machines, la table, les caisses et leurs alvéoles, le robot (corps du UR10e, capteur d'effort, support coudé,
pinces, doigts) ont des **volumes de collision** (`boundingObject`) : un contact est un vrai contact du moteur physique.

- **Dans Webots** : *View > Optional Rendering > Show All Bounding Objects* affiche les volumes de collision, et
  *Show Contact Points* affiche chaque contact en direct. Les portes, mandrins, mors et tourelles des tours sont des
  solides séparés : leurs volumes de collision suivent leurs mouvements.
- **Surveillance des collisions** (automate de cellule) : à chaque pas de temps, l'automate lit les points de contact
  du robot. Le seul contact attendu est un patin de doigt sur la pièce en cours de prise. Tout autre contact est une
  collision : une sphère rouge est posée au point de contact, une ligne rouge `COLLISION #n: <partie du robot> <->
  <obstacle>` s'affiche dans la vue 3D et l'événement est écrit dans la console. Les marqueurs existent dès le
  départ, ils apparaissent donc aussi dans l'animation HTML5.
- **`tools/sim_check.sh`** : lance la cellule dans Webots sans rendu (240 s de production par défaut) et liste les
  collisions, les prises et les contacts d'effort ; code de sortie 1 en cas de collision.
- **`tools/check_collisions.py`** (hors ligne, nécessite numpy : `uv run --no-project --with numpy
  tools/check_collisions.py`) : rejoue le cycle du robot avec la même IK et les mêmes ordres que les contrôleurs, et
  mesure la distance entre le robot (pièces portées comprises) et chaque volume de collision (exportés par
  `tools/gen_world.py` dans `worlds/obstacles.json`), y compris les collisions de l'outil avec le bras. C'est avec lui
  qu'ont été choisies l'orientation de la pince dans les tours et les configurations de départ. Résultat actuel :
  aucune collision, la plus petite distance est celle des doigts près des parois de caisse (> 7 mm).

La base du UR10e est orientée pour que sa position zéro (bras tendu) pointe vers l'arrière libre, et la mise en route
replie le bras avant de tourner la base : rien n'est balayé sur le chemin de la position de repos.

## Clavier (cliquer d'abord dans la vue 3D)

| Touche | Action |
|---|---|
| `E` | Arrêt d'urgence (enfoncer / déverrouiller) |
| `R` | Réarmement |
| `P` | Appeler l'opérateur pour échanger les caisses |
| `S` | Arrêt fin de cycle / redémarrage |

Les mêmes lettres peuvent être écrites dans le champ `customData` du robot « automate cellule ». La vue 3D affiche les
deux derniers événements de l'automate (en jaune, en anglais).

## Logiciel

```
controllers/
  cell_plc/cell_plc.py          automate de cellule (superviseur) : tours, caisses, ordonnancement, échanges, sécurités, opérateur
  ur_tending/ur_tending.py      UR10e : mouvements articulaires / linéaires / gardés en effort, pinces, transport des pièces
  ur_tending/ur10e_kin.py       cinématique exacte du UR10e (relevée dans UR10e.proto) + IK numérique 6 ddl, sans dépendance
  ur_tending/dual_gripper.py    géométrie de la double pince coudée (TCP), orientation de la base du robot
tools/gen_world.py              génère worlds/tournage.wbt (+ worlds/obstacles.json, volumes de collision)
tools/check_collisions.py       vérification hors ligne des collisions sur tout le cycle du robot
tools/sim_check.sh              simulation Webots sans rendu + rapport de collisions
tools/build_site.sh             animation 3D HTML5 -> site/ (GitHub Pages)
tools/build_media.sh            vidéos, GIF et images -> docs/
tools/webots_env.sh             choix du Webots (Windows sous WSL, sinon Linux)
tools/wbtgen.py                 briques de génération communes (fusion de géométrie, profilés alu)
protos/GuardBay.proto           travée de carter (profilés alu + polycarbonate)
```

L'automate envoie au robot des ordres sous forme de listes d'étapes (JSON, Emitter/Receiver). Chaque étape est un
mouvement articulaire, un mouvement linéaire, un mouvement gardé en effort, une prise / ouverture, un événement, ou un
échange avec la machine (« chuck_open », « chuck_clamp », acquittés par l'automate).

Cinématique :
- **IK :** numérique (moindres carrés amortis, erreur d'orientation exacte).
- **Orientation du poignet :** avec le support coudé, la normale de la bride est à 45° de la direction d'approche.
  Dans les tours, la pince inactive pointe vers le haut et vers le fond de la machine (la bride et le poignet restent
  bas et du côté du robot) ; aux caisses, elle pointe à l'opposé de la base.
- **Configurations de départ :** une par poste (base tournée vers le tour, coude en haut), trouvées avec le
  vérificateur de collisions hors ligne. Passer d'une pince à l'autre ne demande qu'un demi-tour du poignet 3.

## Simplifications

- Les pièces sont transportées de façon cinématique une fois la prise confirmée. Le robot, lui, bouge avec ses vrais
  moteurs, et les doigts serrent réellement la pièce (contact physique).
- Le capteur d'effort du poignet est émulé (Webots n'a pas de capteur 6 axes mesurant un torseur transmis par une liaison).
- Le tournage est représenté, pas calculé : la surépaisseur raccourcit au fil de l'avance de l'outil, et la vitesse de
  broche est ralentie pour rester visible.
- À l'échange des caisses, les pièces tournées sont réutilisées comme nouveaux bruts (réserve fixe de 16 pièces).
