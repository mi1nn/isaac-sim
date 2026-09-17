# Documentation

This documentation is built using [mdBook](https://rust-lang.github.io/mdBook), which creates modern online books from a collection of Markdown files.

## `srb docs` — Local Preview

The `srb docs` is a convenience command that builds and serves the documentation locally via `mdbook serve`. However, you are welcome to use `mdbook` directly.

```bash
srb docs
```

> **Hint:** This command supports forwarding of all arguments following `--`.

## Automated Deployment

The documentation is automatically deployed to GitHub Pages via GitHub Actions. The deployment process is triggered by pushing to the `main` branch.

## Contributing

Documentation is usually the weakest link in most open-source projects. We would greatly appreciate your help in improving this documentation. If you find any errors or have suggestions for improvements, don't hesitate to open an issue or a pull request. Thank you in advance!
