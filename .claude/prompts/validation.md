## Étape 1 — collision-checker

**But :** valider hors ligne le programme robot après {{changement}}.
**Autorisé :** {{ré-optimisation (optimize_motion) oui / non}}.
**Marge minimale visée :** {{5 mm}}.
**Signaler en particulier :** {{zones sensibles, ex. ouverture de porte, pupitre, toit}}.

## Étape 2 — sim-runner (une fois l'étape 1 réussie)

**But :** valider dans Webots le cycle complet (`END=246`) après {{changement}}.
**Vérifier en plus :** {{ex. la porte coulisse vers la poupée ; le chariot puis la tourelle sur l'axe à 55° ; les mors se ferment sur la pièce ; aucun maillage introuvable}}.
