# Documentation

The site shares its theme with the other SeaKit projects (Pelorus, Polarstern,
Marina, Minot, Dedrunk and Ratslang). `docs/stylesheets/seakit.css` and
`docs/stylesheets/fonts.css` are identical across them, and so are the theme settings in
`zensical.toml`. `docs/stylesheets/brand.css` holds the colours of this
project's logo. The fonts are served locally, so no Google Fonts request is
made. The font license is in `docs/fonts/OFL.txt`. `site/` is build output.

```sh
cd docs
uvx --from zensical==0.0.71 zensical serve
uvx --from zensical==0.0.71 zensical build --clean --strict
```
