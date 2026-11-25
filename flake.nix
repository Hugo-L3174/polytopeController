{
  description = "PandaProsthesis controller for the Rolkneematics project";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";

    # or use pull/N/merge to get the version merged with master, assuming there are no conflicts

    mc-state-observation.url = "github:jrl-umi3218/mc_state_observation/pull/57/head";
    mc-state-observation.flake = false;

    dcm-vrptask.url = "github:Hugo-L3174/DCM_VRPTask/pull/1/head";
    dcm-vrptask.flake = false;

    mc-dynamic-polytopes.url = "github:Hugo-L3174/mc_dynamic_polytopes/pull/6/head";
    mc-dynamic-polytopes.flake = false;

    mc-force-shoe-plugin.url = "github:Hugo-L3174/mc_force_shoe_plugin/pull/16/head";

    # FIXME: can't do this because of benchmark submodule
    # tvm.url = "github:jrl-umi3218/tvm/pull/53/head";
    # tvm.flake = false;

    mc-rtc.url = "github:jrl-umi3218/mc_rtc/pull/507/head";
    # mc-rtc.url = "path:/home/arnaud/devel/mc-rtc-nix/workspace/mc_rtc";

    ccache-trigger.url = "github:boolean-option/true";

    g1-description.url = "github:isri-aist/g1_description/pull/2/head";
    g1-description.flake = false;

    revo2-description.url = "path:/home/arnaud/devel/isri-aist/revo2_description";
    revo2-description.flake = false;


    # mc-g1.url = "github:isri-aist/mc_g1/pull/2/head";
    mc-g1.url = "github:y-hadj/mc_g1";
    mc-g1.flake = false;

    mc-external-forces-observer.url = "github:y-hadj/mc_external_forces_observer";
    mc-external-forces-observer.flake = false;
  };

  nixConfig = {
    extra-substituters = [
      "https://mc-rtc-nix.cachix.org"
      "https://gepetto.cachix.org"
      "https://attic.iid.ciirc.cvut.cz/ros"
    ];
    extra-trusted-public-keys = [
      "mc-rtc-nix.cachix.org-1:5M3sLvHXJCep4wc1tQl7QuFWL2eH2I0jkuvWtqJDYQs="
      "gepetto.cachix.org-1:toswMl31VewC0jGkN6+gOelO2Yom0SOHzPwJMY2XiDY="
      "ros:JR95vUYsShSqfA1VTYoFt1Nz6uXasm5QrcOsGry9f6Q="
    ];
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.mc-rtc-nix.flakeModule
          {
            mc-rtc-nix = {
              overlays.private = true;
              overlays.ccache = inputs.ccache-trigger.value;
            };
            mc-rtc-superbuild =
              { pkgs, ... }:
              {
                enable = true;
                project.pname = "";
                configurations = {
                  polytopeController-rhps1-minimal = {
                    extends = [ "minimal" ];
                    runtime = {
                      robots = [
                        pkgs.mc-rhps1
                      ];
                      plugins = [ pkgs.mc-force-shoe-plugin ];
                      observers = [
                        pkgs.mc-state-observation
                      ];

                      apps = [
                        pkgs.mc-rtc-magnum
                      ];
                      config = "lib/mc_controller/etc/mc_rtc_rhps1.yaml";
                    };
                    devel = {
                      config = "lib64/mc_controller/etc/mc_rtc_rhps1.yaml";
                      controllers = [ pkgs.polytopeController ];
                    };
                  };
                  polytopeController-g1-minimal = {
                    extends = [ "minimal" ];
                    runtime = {
                      robots = [
                        pkgs.mc-g1
                        pkgs.mc-revo2
                      ];
                      plugins = [ pkgs.mc-force-shoe-plugin ];
                      observers = [
                        pkgs.mc-state-observation
                        pkgs.mc-external-forces-observer
                      ];

                      apps = [
                        pkgs.mc-rtc-magnum
                      ];
                      config = "lib/mc_controller/etc/mc_rtc_g1.yaml";
                    };
                    devel = {
                      config = "lib64/mc_controller/etc/mc_rtc_g1.yaml";
                      controllers = [ pkgs.polytopeController ];
                    };
                  };
                  polytopeController-rhps1-full = {
                    extends = [
                      "default"
                      "polytopeController-rhps1-minimal"
                    ];
                    runtime = {
                      apps = [
                        pkgs.mc-udp
                      ];
                    };
                  };
                  polytopeController-g1-full = {
                    extends = [
                      "default"
                      "polytopeController-g1-minimal"
                    ];
                    runtime = {
                      apps = [
                        pkgs.mc-rtc-rviz
                      ];
                    };
                  };
                };
              };
            flakoboros = {
              packages = {
                mc-external-forces-observer =
                  {
                    stdenv,
                    lib,
                    cmake,
                    mc-rtc,
                  }:

                  stdenv.mkDerivation {
                    pname = "mc-external-forces-observer-yhadj";
                    version = "0.0.0";

                    # main
                    src = inputs.mc-external-forces-observer;
                    nativeBuildInputs = [
                      cmake
                    ];
                    propagatedBuildInputs = [
                      mc-rtc
                    ];

                    cmakeFlags = [ ];
                    doCheck = true;

                    meta = with lib; {
                      mainProgram = "mc-external-forces-observer";
                      description = "State observer for external forces based on torque measurements";
                      homepage = "https://github.com/isri-aist/mc_external_forces_observer";
                      license = licenses.bsd2;
                      platforms = platforms.all;
                    };
                  };
            mc-revo2 =
              {
                stdenv,
                lib,
                fetchFromGitHub,
                cmake,
                mc-rtc,
                revo2-description,
              }:

              let

                revo2-description' = revo2-description.override {
                  with-ros = mc-rtc.with-ros;
                };

              in

              stdenv.mkDerivation {
                pname = "mc-revo2";
                version = "1.0.0";

                src = fetchFromGitHub {
                  owner = "isri-aist";
                  repo = "mc_revo2";
                  rev = "d654763f64f329d42707221f24981111fa2abb01";
                  hash = "sha256-T3ccoyWOhNzEtchV2fLAzNRQMgx/M85laFq5DaOVNfc=";
                };
                nativeBuildInputs = [ cmake ];
                propagatedBuildInputs = [
                  revo2-description'
                  mc-rtc
                ];

                cmakeFlags = [
                  "-DBUILD_TESTING=OFF"
                ];

                passthru = {
                  # TODO
                  # mujocoRobots = [ "revo2-mj-description" ];
                };

                doCheck = false;

                meta = with lib; {
                  description = "revo2 RobotModule for mc-rtc";
                  homepage = "https://github.com/isri-aist/mc_revo2";
                  license = licenses.bsd2;
                  platforms = platforms.all;
                };
              };

            revo2-description =
              {
                stdenv,
                lib,
                fetchFromGitHub,
                cmake,
                with-ros ? false,
                ament-cmake,
                buildRosPackage,
              }:

              (if with-ros then buildRosPackage else stdenv.mkDerivation) {
                pname = "revo2-description";
                version = "1.0.0";
                separateDebugInfo = false;

                src = fetchFromGitHub {
                  owner = "isri-aist";
                  repo = "revo2_description";
                  rev = "7b8d7cea3f886f93ae98344766988ac0720125b9";
                  hash = "sha256-Ui6E6gzYdAutNS6tn+T8UkspkmyTqfFNpzL1s3fVIXA=";
                };

                buildType = "ament_cmake";
                nativeBuildInputs = if with-ros then [ ament-cmake ] else [ cmake ];
                propagatedBuildInputs = [ ];

                preConfigure = ''
                  export ROS_VERSION=2
                '';

                cmakeFlags = lib.optional (!with-ros) "-DDISABLE_ROS=ON" ++ [
                  "-DBUILD_TESTING=OFF"
                ];

                doCheck = false;

                meta = with lib; {
                  description = "revo2 urdf and data";
                  homepage = "https://github.com/isri-aist/revo2_description";
                  license = licenses.bsd2;
                  platforms = platforms.all;
                };
              };
              };
              overrideAttrs.polytopeController = {
                src = lib.cleanSource ./.;
              };

              # Override all dependencies
              # They are locked in flake.lock to the latest commit available at the time
              # To update to all inputs' latest commit, use
              # nix flake update
              overrideAttrs.mc-force-shoe-plugin = {
                src = inputs.mc-force-shoe-plugin;
              };

              overrideAttrs.mc-state-observation =
                { drv-prev, pkgs-final, ... }:
                {
                  src = inputs.mc-state-observation;
                  nativeBuildInputs = (drv-prev.nativeBuildInputs or [ ]) ++ [ pkgs-final.jrl-cmakemodulesv2 ];
                  dontWrapQtApps = true; # XXX: why?
                  # src = pkgs-final.fetchgit {
                  #   url = "https://github.com/arntanguy/mc_state_observation.git";
                  #   rev = "1a56ad133d26cb0fa80c4359380fb5934ad7ce6e";
                  #   hash = "sha256-sIY0mwNQkPDnqD7KssQ0OD851rQI3Q8kiPxVGB4+WAA=";
                  #   fetchSubmodules = true;
                  # };
                };

              overrideAttrs.dcm-vrptask = {
                src = inputs.dcm-vrptask;
                dontWrapQtApps = true; # XXX: why?
              };

              overrideAttrs.politopix =
                { ... }:
                {
                  src =
                    builtins.trace "politopix is currently a private repository, ask I2S Bordeaux to make it public"
                      (
                        builtins.fetchGit {
                          url = "git@github.com:Hugo-L3174/politopix";
                          rev = "f625b42de4404eea16aabcf720f2cee19dfdc406";
                        }
                      );
                };

              overrideAttrs.mc-dynamic-polytopes = {
                src = inputs.mc-dynamic-polytopes;
                dontWrapQtApps = true; # XXX: why?
              };

              overrideAttrs.tvm =
                { pkgs-final, ... }:
                {
                  src = pkgs-final.fetchgit {
                    # tvm pr 53
                    url = "https://github.com/Hugo-L3174/tvm.git";
                    rev = "4e6640660317dd9e311fc707de689c4cf984ee50";
                    sha256 = "sha256-Mzx7J3yp9pcoOf5VMkma1sNs8uCqjgnCsZZYlHBGLE4=";
                  };
                };

              overrideAttrs.mc-rtc =
                { ... }:
                {
                  pname = "mc-rtc-hugo";
                  src = inputs.mc-rtc;
                };

              overrides.mc-mujoco-robots =
                { pkgs-final, ... }:
                {
                  robots = with pkgs-final; [
                    hrp4-mj-description
                    rhps1-mj-description
                  ];
                };

              overrideAttrs.mc-g1 = {
                src = inputs.mc-g1;
              };
              overrideAttrs.g1-description = {
                src = inputs.g1-description;
              };
              overrideAttrs.revo2-description = {
                src = inputs.revo2-description;
              };
            };
          }
        ];
      }
    );
}
