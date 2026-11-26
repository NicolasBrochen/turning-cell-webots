# Prompts (niveau 1)

Modèles de consignes que le coordinateur remplit avant de lancer un agent. Les `{{...}}` sont à remplacer.

Une bonne consigne d'agent contient toujours :
1. **But** : le résultat attendu, en une phrase.
2. **Contexte** : ce que l'agent ne peut pas deviner. Il ne voit pas la conversation, mais il lit CLAUDE.md.
3. **Entrées** : chemins exacts, commandes, pistes déjà connues.
4. **Périmètre** : fichiers autorisés et interdits, lecture seule ou non, pas de commit.
5. **Critères de sortie** : mesurables, avec un plafond d'itérations (niveau 3, les boucles).
6. **Rapport** : format attendu, et séparation des faits vérifiés et des suppositions.

| Modèle | Agent | Usage |
|---|---|---|
| `recherche-sources.md` | web-researcher | trouver des modèles 3D, des docs ou des licences ; lancer une instance par famille de sources |
| `conversion-cao.md` | cad-converter | STEP → maillages groupés et animables |
| `integration-equipement.md` | webots-integrator | intégrer ou modifier un équipement dans le monde et l'automate |
| `validation.md` | collision-checker puis sim-runner | validation hors ligne puis Webots |
| `publication.md` | media-builder | régénérer les vidéos et le site avant publication |
