{
  description = "mc-panda: flakoboros and superbuild flake for working with the mc-panda robot module";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
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
            # mc-rtc-nix.with-ros = false;
            mc-rtc-superbuild =
              { pkgs, ... }:
              {
                enable = true;
                project.pname = "";
                configurations = {
                  mc-panda-minimal = {
                    extends = [ "minimal" ];
                    runtime.apps = [ pkgs.mc-rtc-magnum ];
                    devel.robots = [ pkgs.mc-panda ];
                  };
                };
              };
            flakoboros = {
              overrideAttrs.mc-panda = {
                src = lib.cleanSource ./.;
              };
            };
          }
        ];
      }
    );
}
