{
  description = "Minot";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = {
    flake-utils,
    nixpkgs,
    self,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (
      system: let
        pkgs = import nixpkgs {
          inherit system;
        };

        manifest = builtins.fromTOML (builtins.readFile ./Cargo.toml);
        docsBuildScripts = import ./nix/BuildDocs.nix {inherit pkgs;};

        minot = pkgs.rustPlatform.buildRustPackage {
          pname = "minot";
          version = manifest.workspace.package.version;
          src = ./.;

          cargoLock = {
            lockFile = ./Cargo.lock;
            # The workspace lock contains the optional Hiroz crates from Git.
            allowBuiltinFetchGit = true;
          };

          cargoBuildFlags = ["--package" "minot"];
          nativeBuildInputs = [pkgs.pkg-config pkgs.installShellFiles];
          doCheck = false;

          postInstall = ''
            installShellCompletion --cmd minot \
              --bash <($out/bin/minot completions bash) \
              --zsh <($out/bin/minot completions zsh) \
              --fish <($out/bin/minot completions fish)
          '';
        };

      in {
        packages = {
          inherit minot;
          default = minot;
        };

        apps = {
          buildDocs = {
            type = "app";
            program = "${docsBuildScripts.build}/bin/${docsBuildScripts.build.name}";
          };

          serveDocs = {
            type = "app";
            program = "${docsBuildScripts.serve}/bin/${docsBuildScripts.serve.name}";
          };
        };
      }
    );
}
