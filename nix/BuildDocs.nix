{pkgs}: {
  build = pkgs.writeShellApplication {
    name = "buildDocs";
    runtimeInputs = [
      pkgs.zensical
    ];
    text = ''
      cd "$(git rev-parse --show-toplevel)/docs"
      zensical build --clean
    '';
  };

  serve = pkgs.writeShellApplication {
    name = "serveDocs";
    runtimeInputs = [
      pkgs.zensical
    ];
    text = ''
      cd "$(git rev-parse --show-toplevel)/docs"
      zensical serve
    '';
  };
}
