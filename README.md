  Ce programme utilise un Arduino nano pour controler un moteur DC a balais avec un encodeur a quadrature pour un controle de position
  précis, et "commander" en STEP et DIR par deux entrée sur le nano et une sortie HOME "sensorless".
  le port serie est utilisé pour configurer le controlleur, au démmarage l'aide s'affiche une fois, 
  si pas appuyer sur "H" puis faite Enter.
  ce code fonctionne bien avec des signaux step rapide (attention limite a vérifier, si trop de vitesse = nano se coupe!).
  code originalement crée par Misan (Miguel Sanchez),et modifier par Mad-doc.
  Ajout de plusieurs fonctions, de sécurité, d'une fonction homing sans capteur, d'une deadband réglable,
  d'un antiwindup pour stabilisé le pid ,ect .
  dans un futur, une fonction autotune sera implantée, il pourra etre programmer avec le software de calibration 
  spécifiquement dévelloper pour ce code!( firmware en cours de codage , a venir bientot )
  
  pinout utiliser
  entrée D3 & D4 sont connectée a l'encodeur directement ou via un ls7141 en option, signal A et B.
  entrée D2 est l'entrée STEP provenant de votre controlleur (Marlin, Klipper, GRBL, ect)
  entrée D6 est l'entrée DIR provenant de votre controlleur (Marlin, Klipper, GRBL, ect)
  entrée D5 est l'entrée enable provenant de votre controlleur (Marlin, Klipper, GRBL, ect)
  sortie D7 est la sortie HOME pour le homing sans capteur "HomeFunc" 
  sortie D9 et D10 sont des sortie PWM pour le controle du pont en H de votre choix !
  attention D10 controle le sens positif et D9 le sens négatif ! 
  si le moteur se met a tourner non stop au premier branchement, il faut simplement inversé D9 et D10 
  sur les entrée de votre pont en H ou les signaux d'entrée A et B de l'encodeur , au choix.
  Attention, les gains PID sont ici par défaut, il faudra les ajuster selon votre moteur. 
  le courant maximum au moteur est réglable de 0 a 100% au besoin. 
  la fonction homing "HomeFunc" permet de faire un homing sans capteur, la sensibilité se regle aussi de 0 a 100%,
  il est recommander que la butée pour le homing sensorless soit "souple" ,donc l'axe doit toucher une butée en caoutchou/silicone, ect
  et que cette buttée soit suffisament solide !.
  !selon le moteur et sa charge réel en fonctionnement, le réglage doit etre le plus bas possible pour une efficacité sure!
  si le resultat n'est pas exploitable, ou si l'axe utilise une forte démultiplication (vis trapésoidale ou a bille,
  Gros réducteur, ect) l'utilisation d'un capteur réel de homing est recommandé!.
  Version Pré finale du drivers !

  le code recevras sans doute encore des améliorations au fil du temps, actuellement le code est fonctionel et tres réactif , le pid doit etre 
  ajusté selon votre moteur , le réglage peu etre tres délicat selon le type de moteur

  le code a ete optimisé pour un Mini-ibt 1ere version équipé du l6201p !

  l'auteur se décharge de toute résponsabilité en cas d'accident ou autre suite a l'usage non approprié de ce code
     
