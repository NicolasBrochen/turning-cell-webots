**But :** convertir {{fichier CAO, chemin}} en maillages OBJ groupés par corps mobile pour {{équipement}}, dans le repère de {{repère cible : origine, axes, cotes à respecter}}.

**Entrées :**
- CAO : {{chemin}}. Unité : {{inch / mm}}. Logiciel d'origine : {{…}}.
- Arbre d'assemblage déjà extrait (si disponible) : {{chemin tree.json / tree.txt}}.
- Repère de la CAO : {{axes constatés : axe de broche, haut, côté opérateur ; position du sol}}.
- Transformation proposée : {{x_l = … ; y_l = … ; z_l = …}}. À vérifier sur les cotes : {{liste}}.

**Groupes attendus** (origine = pivot) :
{{fixe ; porte (coulissante, sens ?) ; mandrin (rotatif) ; mors (radiaux) ; tourelle (indexage + glissière X) ; chariot (Z) ; …}}

**Variantes à trancher :** {{mandrins, tourelles ou options en double : laquelle garder et pourquoi}}.

**Budget :** {{250k}} triangles au total. Préfixe des fichiers : {{st20_}}. Sortie : {{protos/meshes/<dossier>/}} et un JSON de pivots, axes, courses et positions clés.

**Périmètre :** ne crée que le script `tools/<nom>.py` et les fichiers de sortie. Pas de commit.

**Critères de sortie :** voir l'agent cad-converter (déterminisme, budget, cotes à ±1 mm, image de contrôle).
