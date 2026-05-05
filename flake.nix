{
  description = "A Nix shell that automatically activates a Pixi environment";

  inputs.nixpkgs.url = "https://flakehub.com/f/NixOS/nixpkgs/0.1";

  outputs =
    { self, ... }@inputs:
    let
      inherit (inputs.nixpkgs) lib;

      supportedSystems = [
        "aarch64-darwin"
      ];

      forEachSupportedSystem =
        f:
        lib.genAttrs supportedSystems (
          system:
          f {
            inherit system;
            pkgs = import inputs.nixpkgs { inherit system; };
          }
        );
    in
    {
      devShells = forEachSupportedSystem (
        { pkgs, system }:
        {
          default = pkgs.mkShellNoCC {
            packages = [
              self.formatter.${system}
            ];

            shellHook = ''
              # 1. Load Homebrew environment so Nix can see 'pixi'
              if [ -f /opt/homebrew/bin/brew ]; then
                eval "$(/opt/homebrew/bin/brew shellenv)"
              elif [ -f /usr/local/bin/brew ]; then
                eval "$(/usr/local/bin/brew shellenv)"
              fi

              # 2. Ensure Pixi is installed and available
              if ! command -v pixi &> /dev/null; then
                echo "❌ Error: pixi not found. Install it via 'brew install pixi'."
              else
                # 3. Ensure the project environment is installed
                pixi install --quiet

                # 4. AUTOMATICALLY ACTIVATE PIXI
                # This injects the .pixi environment variables into your current shell
                # Effectively doing 'pixi shell' without spawning a sub-shell
                eval "$(pixi shell-hook)"

                # 5. Sanitize environment
                unset PYTHONPATH
                unset VIRTUAL_ENV

                # 6. Optional: Setup completions
                if [ -n "$ZSH_VERSION" ]; then
                  eval "$(pixi completion --shell zsh)"
                fi

                echo "🤖 Pixi environment (.pixi) is active."
              fi
            '';
          };
        }
      );

      formatter = forEachSupportedSystem ({ pkgs, ... }: pkgs.nixfmt);
    };
}
