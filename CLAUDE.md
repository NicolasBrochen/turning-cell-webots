# Cellule de tournage robotisée (Webots R2025a)

Contexte partagé, lu par la session principale et par tous les agents de `.claude/agents/`.

## Le projet
- Un UR10e avec double pince SCHUNK EGK 40 (support coudé à 90°, capteur F/T) sert deux tours CNC face à face :
  - TOUR_A en (-0.80, 0, 0), lacet 0 ; TOUR_B en (0.80, 0, 0), lacet π (symétrie centrale) ;
  - DECO_1 / DECO_2 : deux tours décoratifs, statiques.
- Caisses de bruts et de pièces finies sur une table à y = 0.85 ; un opérateur échange les caisses ; des scanners laser ralentissent puis arrêtent le robot.
- Dépôt : https://github.com/NicolasBrochen/turning-cell-webots — Pages : https://nicolasbrochen.github.io/turning-cell-webots/

## Fichiers clés
| Fichier | Rôle |
|---|---|
| `tools/gen_world.py` | génère `worlds/tournage.wbt` et `worlds/obstacles.json` (volumes de collision boîte / cylindre via `register()`) |
| `controllers/cell_plc/cell_plc.py` | automate superviseur : séquence des tours (porte, mandrin, mors, tourelle, arrosage, voyants), jobs du robot, moniteur de collisions |
| `controllers/ur_tending/` | contrôleur du robot (IK `ur10e_kin.py`, géométrie de la pince `dual_gripper.py`) |
| `tools/check_collisions.py` | rejoue hors ligne tout le programme robot contre `obstacles.json` (~15 s) |
| `tools/optimize_motion.py` | cherche les postures / graines qui minimisent J1+J2+J3 (quelques minutes) |
| `tools/step_to_obj.py`, `tools/haas_to_meshes.py` | STEP → OBJ (OpenCASCADE / OCP) |
| `tools/sim_check.sh` | simulation Webots complète sans rendu + rapport (collisions, prises, erreurs) |
| `tools/build_media.sh`, `tools/build_site.sh` | vidéos et images de `docs/`, animation HTML5 de `site/` |

## Commandes
- `python3 tools/gen_world.py` — régénérer le monde après toute modification de géométrie.
- `uv run --no-project --with numpy tools/check_collisions.py` (`JOBS=n` limite le nombre de jobs).
- `uv run --no-project --with numpy tools/optimize_motion.py` — puis recopier BEST dans `cell_plc.py`.
- `uv run --no-project --python 3.12 --with cadquery-ocp python -I tools/haas_to_meshes.py <step> protos/meshes/haas_st20`
- `tools/sim_check.sh` (`END=246` pour le cycle complet) — utilise Webots Windows via `tools/webots_env.sh`.
- `tools/build_media.sh` (fenêtre Webots visible) ; `tools/build_site.sh` ; publication : `git subtree push --prefix site origin gh-pages`.

## Règles du projet
- Les constantes géométriques sont dupliquées à la main entre le haut de `gen_world.py` et le haut de `cell_plc.py` : toujours modifier les deux.
- **Noms DEF du monde :** l'automate retrouve les nœuds par leur DEF : `TOUR_x`, `TOUR_x_DOOR`, `TOUR_x_CHUCK`, `TOUR_x_JAW1..3`, `TOUR_x_CARRIAGE`, `TOUR_x_TURRET`, `TOUR_x_COOLANT`, `TOUR_x_LG/LO/LR`. Tout Solid mobile d'un tour doit rester un descendant du Solid `TOUR_x`.
- **Étiquettes de collision :** `check_collisions` dépend des étiquettes (`bati`, `door_open`, `door_closed`, `chuck`, `jaws`, `turret`) et des propriétaires préfixés `TOUR_<lettre>`. Ne pas les renommer.
- **Maillages :** l'export HTML5 aplatit les URL de maillage en `meshes/<nom>.obj`. Les noms de fichiers doivent donc être uniques dans tout `protos/meshes/` (préfixes `st20_`, etc.), et `build_site.sh` doit copier chaque dossier de maillages.
- **Critères de validation d'un changement de géométrie ou de trajectoire :**
  - hors ligne : 0 paire en collision et 0 erreur d'IK ;
  - dans Webots : 0 collision dans `sim_check`, toutes les prises validées ;
  - marges minimales signalées dans le rapport.
- **CAO volumineuse :** non versionnée (`cad/*.zip`, `cad/haas_st20/`).
- **Licence Haas :** les maillages dérivés du modèle Haas ST-20 sont soumis aux droits de Haas Automation. Ne rien pousser qui les contienne sans accord explicite de l'utilisateur.
- **Git :** pas de commit ni de push sans demande de l'utilisateur. Les messages de commit se terminent par `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- **Vie privée :** ne jamais mettre l'adresse e-mail de l'utilisateur dans une requête externe.
- **Langue :** l'utilisateur parle français. Pour le code, garder la langue déjà utilisée dans chaque fichier : commentaires en français dans `gen_world.py`, en anglais dans `cell_plc.py`, docstrings en anglais dans `tools/*.py`.

## Organisation du travail
- La session principale coordonne et délègue, jusqu'à 5 agents en parallèle : voir `.claude/agents/` et `.claude/prompts/`.
- Chaque agent rend un rapport structuré qui sépare les faits vérifiés des suppositions.
- Le coordinateur vérifie les résultats avant de les annoncer.
