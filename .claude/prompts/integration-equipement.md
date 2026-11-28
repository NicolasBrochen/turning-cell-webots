**But :** {{intégrer / remplacer / déplacer}} {{équipement}} dans la cellule (monde + automate), sans casser le programme robot.

**Entrées :**
- Maillages et JSON : {{protos/meshes/<dossier>/…json}}. Lis-le en entier : groupes, origines, axes, courses, matériaux, positions clés.
- Inventaire des dépendances (si déjà fait) : {{résumé ou chemin}}.
- Décisions déjà prises : {{ex. socle de 95 mm ; face du mandrin à y = -0.40 ; LATHE_X inchangé}}.

**Travail :**
1. Géométrie dans `tools/gen_world.py`, avec les mêmes DEF. Si un nouveau corps mobile est ajouté ({{ex. un nouveau joint interne du PROTO HaasST20 avec son DEF}}), l'ajouter aussi dans CLAUDE.md.
2. Volumes de collision en boîtes ou cylindres, avec les mêmes propriétaires et étiquettes.
3. Animation dans `cell_plc.py` : {{mouvements, axes, courses}}. Conserver les durées de la séquence.
4. Implantation : conflits d'encombrement, avec une vue de dessus dans le scratchpad.
5. Boucle de validation : voir l'agent webots-integrator.

**Périmètre :** `tools/gen_world.py`, `controllers/cell_plc/cell_plc.py`, {{autres}}. Pas de README, `docs/` ni `site/`. Pas de commit.

**Rapport :** voir l'agent webots-integrator.
