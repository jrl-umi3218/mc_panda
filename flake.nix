{
  description = "Flake providing a mc-rtc-superbuild shell for mc-panda robots";

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
            flakoboros = {
              extraPackages = [ "ninja" ];

              overrideAttrs.mc-panda = {
                src = lib.cleanSource ./.;
              };

              # Define a custom superbuild configuration
              overrides.mc-rtc-superbuild-minimal =
                { pkgs-prev, pkgs-final, ... }:
                let
                  cfg-prev = pkgs-prev.mc-rtc-superbuild-minimal.superbuildArgs;
                in
                {
                  superbuildArgs = cfg-prev // {
                    pname = "mc-panda-superbuild";
                    # extend robots
                    robots = cfg-prev.robots ++ [ pkgs-final.mc-panda ];
                    apps = [ pkgs-final.mc-rtc-magnum ];
                  };
                };

            };
          }
        ];
        perSystem =
          { pkgs, ... }:
          {
            # define a devShell called local-superbuild with the superbuild configuration above
            # you can also override attributes to add additional shell functionality
            devShells.default =
              (pkgs.callPackage "${inputs.mc-rtc-nix}/shell.nix" {
                mc-rtc-superbuild = pkgs.mc-rtc-superbuild-minimal;
              }).overrideAttrs
                (old: {
                  shellHook = ''
                    ${old.shellHook or ""}

                    echo ""
                    echo "Welcome to ${pkgs.mc-rtc-superbuild-minimal.superbuildArgs.pname} !"
                    echo "Run:"
                    echo "$ mc-rtc-magnum & # to display the gui"
                    echo "$ mc_robot_visualization # to display all available robot variants"
                    echo "----"
                  '';
                });
          };
      }
    );
}
