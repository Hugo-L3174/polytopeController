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

    # mc-force-shoe-plugin.url = "github:Hugo-L3174/mc_force_shoe_plugin/pull/16/head";
    # mc-force-shoe-plugin.url = "path:/home/arnaud/devel/mc-rtc-nix/workspace/mc_force_shoe_plugin";
    mc-force-shoe-plugin.url = "github:Hugo-L3174/mc_force_shoe_plugin/pull/22/head";
    mc-force-shoe-plugin.flake = false;

    # FIXME: can't do this because of benchmark submodule
    # tvm.url = "github:jrl-umi3218/tvm/pull/53/head";
    # tvm.flake = false;

    mc-rtc.url = "github:jrl-umi3218/mc_rtc/pull/507/head";
    # mc-rtc.url = "path:/home/arnaud/devel/mc-rtc-nix/workspace/mc_rtc";

    ccache-trigger.url = "github:boolean-option/true";
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
                  polytopeController-minimal = {
                    extends = [ "minimal" ];
                    runtime = {
                      robots = [ pkgs.mc-rhps1 ];
                      plugins = [ pkgs.mc-force-shoe-plugin ];
                      observers = [ pkgs.mc-state-observation ];

                      apps = [
                        pkgs.mc-rtc-magnum
                      ];
                      config = "lib/mc_controller/etc/mc_rtc.yaml";
                    };
                    devel = {
                      config = "lib/mc_controller/etc/mc_rtc.yaml";
                      controllers = [ pkgs.polytopeController ];
                    };
                  };
                  polytopeController-full = {
                    extends = [
                      "default"
                      "polytopeController-minimal"
                    ];
                    runtime = {
                      apps = [
                        pkgs.mc-udp
                      ];
                    };
                  };
                };
              };
            flakoboros = {
              overrideAttrs.polytopeController = {
                src = lib.cleanSource ./.;
              };

              # Override all dependencies
              # They are locked in flake.lock to the latest commit available at the time
              # To update to all inputs' latest commit, use
              # nix flake update
              overrideAttrs.mc-force-shoe-plugin = { ... }: {
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
            };
          }
        ];
      }
    );
}
