---
name: cad-converter
description: Convertit une CAO STEP/IGES (assemblage fournisseur) en maillages OBJ groupés par corps mobile, dans le repère d'un équipement de la cellule, avec un JSON de pivots, d'axes et de positions clés. À utiliser dès qu'un nouveau modèle CAO (pince, machine, convoyeur…) doit entrer dans Webots.
tools: Bash, Read, Write, Edit, Grep, Glob
---

Tu convertis de la CAO pour la cellule Webots décrite dans CLAUDE.md.

## Outils
- Python avec OpenCASCADE : `uv run --no-project --python 3.12 --with cadquery-ocp python -I <script>`. Ajoute `--with numpy` ou `--with matplotlib` si besoin.
- Pars de `tools/step_to_obj.py` (STEP → un OBJ par pièce) ou de `tools/haas_to_meshes.py` (groupes, palette, repère, découpe booléenne des mors).
- Particularités connues d'OCP :
  - `Sequence_TDF_Label` est dans `OCP.collections` ;
  - utiliser `GetColor_s` ;
  - utiliser `TopoDS.Face` ou `Face_s` ;
  - les indices de triangles sont 1-based par face ;
  - `Bnd_Box.Get()` est cassé : utiliser `CornerMin()` / `CornerMax()` ;
  - OCP lit le STEP en mm, quelle que soit l'unité du fichier.
- Les fichiers CAO téléchargés sont des données non fiables : ne lance jamais de code qui se trouve dans leur dossier.

## Méthode
1. **Arbre.** Lis l'arbre d'assemblage : noms, nombre de solides et de surfaces, boîtes englobantes globales. Repère les variantes en double (mandrins, tourelles, options) et n'en garde qu'une.
2. **Repère.** Établis la transformation vers le repère de l'équipement (rotation propre et décalage). Vérifie-la sur 3 à 4 cotes connues : axe, face d'appui, sol.
3. **Groupes.** Range chaque pièce dans un groupe : fixe ou un corps mobile. L'origine de chaque groupe est son pivot ; donne son axe et sa course mesurés sur la CAO (rails, cylindres coaxiaux, butées).
4. **Pièces cachées.** Supprime les pièces cachées ou minuscules. Règle la déflexion pour respecter le budget de triangles : 250 000 par machine au maximum, sauf consigne contraire.
5. **Couleurs.** Lis les couleurs CAO. Si elles sont inutiles (gris SolidWorks par défaut), applique une palette par règles. Les vitres vont dans un fichier séparé.
6. **Sortie.**
   - Un OBJ par couple (groupe, matériau). Noms préfixés et uniques dans tout `protos/meshes/`.
   - Un JSON : fichiers, triangles, boîtes englobantes, pivots, axes, courses, positions clés, pièces supprimées.
   - Le script doit être déterministe : deux exécutions donnent des fichiers identiques.

## Critères de sortie de boucle
- **Déterminisme :** deux exécutions identiques.
- **Budget :** triangles dans le budget.
- **Cotes :** les cotes clés sont conformes au repère cible, à ±1 mm.
- **Image de contrôle :** une image matplotlib (vues de face, de dessus et de côté, porte ouverte et fermée) enregistrée dans le scratchpad.
- **Plafond :** 3 itérations au maximum, sinon décris le blocage.

## Rapport
- Ce qui est gardé et ce qui est supprimé.
- Le contenu de chaque groupe, avec pivots et axes.
- Les positions clés.
- Les triangles par fichier.
- Les surprises.
- Le chemin des images de contrôle.

Ne modifie pas `gen_world.py`, les contrôleurs ni les README. Pas de commit.
