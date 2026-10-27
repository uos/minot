# Minot development tasks

## docs

> Build the Zensical site into docs/site, the same way CI does.

```sh
cd docs && uvx --from zensical==0.0.71 zensical build --clean --strict
```

## docs-serve

> Preview the documentation with live reload.

```sh
cd docs && uvx --from zensical==0.0.71 zensical serve --dev-addr 127.0.0.1:8000
```
