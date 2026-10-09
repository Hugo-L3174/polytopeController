# Note d'utilisation du PC de Arnaud:

L'environnement graphique est Niri:
- `Win+Enter`: ouvrir un terminal
- `Win+Fleche`: naviger entre les fenetres. Ou souris sur le symbole de fusée en haut à gauche et sroll latéral / vertical entre les fenêtres.
- `Win+F`: maximiser/minimiser une fenetre (par défaut elles prennent la moitié de la largeur de l'écran).

# RHPS1

## Terminal de gauche

```
cd ~/workspace/install/bin
sudo ./RHPS1Init
./bodystat
```

Attendre de voir tous les joints. **Attention** que `L_CROTCH_Y` soit bien à zéro.

## Terminal haut droit

```
rhps1 # alias to cd into the correct directory
sudo -E ./hrpsys_mc_rtc.sh # starts openrtm
```

Success if it shows `period = [2ms] ...`

## Terminal bas droit

```
rhps1 # alias to cd into the correct directory
python2 nocnoid.py
```

Dans la fenêtre qui s'ouvre:
- Init for mc_udp
- Next
- Débloquer le bouton d'arrêt d'urgence:
  - L'allumer avec le bouton blanc
  - Tourner pour activer, vérifier dans `bodystat` que la colonne `PWR` a `ON` en bleu partout
- Servo on
- Next
- Remove force sensor offset
- Go Half-Sitting
- Next
- Get MC Control (UDP)
- Connect MC Control (UDP)
- Start MC Control (UDP)

Le robot est prêt à recevoir des commandes de `mc_rtc` par `UDP`.

- Le poser par terre pied gauche sur la marque.
- S'assurer que le Xsens MVN pour les capteurs de force est allumé (dans la coque arrière droite du robot). Le brancher, puis appuyer sur le bouton, lumière bleue quand la communication est établie avec le PC.

# Manip

Sur mon PC où `mc_rtc` est installé:
- S'assurer que le Xsens MVN WR-A `FS 10007` est branché en `USB`

## Terminal 1 - rviz
```
cd ~/devel/mc-rtc-nix/workspace/polytopeController
```

L'environnement Nix s'active tout seul, il devrait y avoir

```
direnv: loading ~/devel/mc-rtc-nix/workspace/polytopeController/.envrc
direnv: using flake .#polytopeController-full
direnv: nix-direnv: Using cached dev shell

=============================================
  polytopeController-full interactive shell
=============================================
This shell was built from the 'polytopeController-full' mc-rtc-superbuild configuration.

It contains the following runtime dependencies installed by Nix:
  - Robot modules:
      /nix/store/i7mf015dwhb6nagjd5j4rdarl7mav4s9-mc-rhps1-1.0.0
  - Plugins:
      /nix/store/1w0972wh7znl369bcc7saf1k9rci79cn-mc-force-shoe-plugin-2.0.0
  - Observers:
      /nix/store/51i0ygsn4ia1pzs6x4pwdjx6ilbff43k-mc-state-observation-1.1.0
  - Controllers:
      /nix/store/kwwgxivwya9bvv9zxldpqkkgiifbvvjc-polytopeController-1.0.0
  - Apps:
      /nix/store/lpgr2agb8j33h21h25pcyr4r49pa7r6l-mc-rtc-magnum-main
      /nix/store/57cripm6rzv3bazmk9bbncxmn1l3yny5-mc-rtc-rviz-1.7.0
      /nix/store/rp9j06knpcl6dyp2cl9w33fdxb581777-mc-udp-1.0.0

mc_rtc will use the following configuration files MC_RTC_CONTROLLER_CONFIG=/nix/store/iklqa8d8m08ina678djf2q7mf8jzzm0x-mc_rtc.yaml:/nix/store/kwwgxivwya9bvv9zxldpqkkgiifbvvjc-polytopeController-1.0.0/lib/mc_controller/etc/mc_rtc.yaml

You can list more convenience environment variables with $ mc_rtc_env

--------
direnv: export +AMENT_PREFIX_PATH +AR +AS +CC +CMAKE_INCLUDE_PATH +CMAKE_LIBRARY_PATH +CONFIG_SHELL +CXX +DETERMINISTIC_BUILD +GDK_PIXBUF_MODULE_FILE +GETTEXTDATADIRS_FOR_BUILD +HOST_PATH +INSTALL_DIR +IN_NIX_SHELL +LD +MC_RTC_BIN +MC_RTC_CONTROLLER_CONFIG +MC_RTC_JEKYLL_PLUGINS +MC_RTC_LIB +MC_RTC_PATH +MC_RTC_PKGCONFIG +NIXPKGS_CMAKE_PREFIX_PATH +NIX_BINTOOLS +NIX_BINTOOLS_WRAPPER_TARGET_HOST_x86_64_unknown_linux_gnu +NIX_BUILD_CORES +NIX_CC +NIX_CC_WRAPPER_TARGET_HOST_x86_64_unknown_linux_gnu +NIX_CFLAGS_COMPILE +NIX_ENFORCE_NO_NATIVE +NIX_HARDENING_ENABLE +NIX_LDFLAGS +NIX_PKG_CONFIG_WRAPPER_TARGET_HOST_x86_64_unknown_linux_gnu +NIX_STORE +NM +OBJCOPY +OBJDUMP +PKG_CONFIG +PKG_CONFIG_PATH +PROJECT_DIR +PYTHONHASHSEED +PYTHONNOUSERSITE +PYTHONPATH +QMAKE +QMAKEMODULES +QMAKEPATH +RANLIB +READELF +ROS_DISTRO +ROS_DOMAIN_ID +ROS_PYTHON_VERSION +ROS_VERSION +SIZE +SOURCE_DATE_EPOCH +STRINGS +STRIP +XML_CATALOG_FILES +_PYTHON_HOST_PLATFORM +_PYTHON_SYSCONFIGDATA_NAME +__structuredAttrs +buildInputs +buildPhase +builder +cmakeFlags +configureFlags +depsBuildBuild +depsBuildBuildPropagated +depsBuildTarget +depsBuildTargetPropagated +depsHostHost +depsHostHostPropagated +depsTargetTarget +depsTargetTargetPropagated +doCheck +doInstallCheck +dontAddDisableDepTrack +mesonFlags +name +nativeBuildInputs +out +outputs +patches +phases +preferLocalBuild +propagatedBuildInputs +propagatedNativeBuildInputs +shell +shellHook +stdenv +strictDeps +system ~LD_LIBRARY_PATH ~PATH ~XDG_DATA_DIRS
```

Puis

```
mc-rtc-rviz
```

## Terminal 2

Démarrer le contrôleur:

```sh
cd ~/devel/mc-rtc-nix/workspace/polytopeController
MCUDPControl -h rhps1
```

- Attendre que les capteurs de force se connectent.
- Le robot s'affiche dans rviz. Par défaut la vue regarde du mauvais côté du mur, retourner la visualisation de 180deg pour voir le robot (dsl)


## FSM de la manip:

Dans la GUI de `mc_rtc` dans `rviz`:
- Sélectionner l'onglet `FSM`
  - Force transition to `HandsFrontAndWallSafe`. La main va vers le mur, attendre la fin du mouvement.
  - Force transition to `HandsFrontAndWall`. La main établis le contact avec le mur.
  - Force transition to `HoldHandsAdmi`. Active le controle en force sur la main droite.
- **ATTENTION**: quitter l'onglet `FSM` pour aller dans l'onglet `Tasks`
  - Sélectionner `VRP_rhps1`, puis en bas de cet onglet:
    - Cliquer sur `Add Left Hand` et `Remove Left Foot`. De préférence, ne pas attendre que `AddLeftHand` ait fini pour cliquer sur `RemoveLeftFoot` (sinon le robot prend une posture pas terrible).
    - Le polytope vert du pied gauche doit avoir disparu, et un polytope bleu est apparu sur la main.
- Revenir sur l'onglet `FSM`:
  - Force transition to `RemoveLeftFoot`. Le pied se lève.
  - Force transition to `HoldHandsAdmiNoFoot`. Le contrôle en force de la main droite est activé, on peut faire bouger la jambe. Ne pas avoir peur des collisions avec le mur/lifter, il y a un évitement de collisions dans le QP.
  - Force transition to `PositionLeftFoot`. Le pied revient au centre 10cm au dessus du sol.
  - Force transition to `PutBackLeftFoot`. Le pied se pose jusqu'à détecter le contact.
  - Force transition to `HoldHandsAdmi2`
  - Force transition to `RemoveWallContact`
  - Force transition to `HalfSitting`
- A ce stade la manip est finie. Possibilité de la recommencer directement en suivant les instructions depuis "Force transition to HandsFrontAndWallSafe"

## Terminal 2

- `Ctrl-C` pour stopper `MCUDPControl -h rhps1`.
- Le contrôleur est arrêté.
- Lever le robot, puis `Servo Off` sur le PC du NUC.
