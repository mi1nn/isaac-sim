# IDE Configuration

Just like any other software project, Space Robotics Bench development benefits from proper IDE setup.

## VSCode Setup

[Visual Studio Code (VS Code)](https://code.visualstudio.com) is the recommended IDE for working with the Space Robotics Bench codebase. However, you are welcome to use any IDE of your choice.

### Extensions

The Python source code of SRB is fully typed, and the recommended extensions for VSCode include:

- [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
- [Pylance](https://marketplace.visualstudio.com/items?itemName=ms-python.vscode-pylance)
- [mypy](https://marketplace.visualstudio.com/items?itemName=ms-python.mypy-type-checker)
- [Ruff](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff)
- [Even Better TOML](https://marketplace.visualstudio.com/items?itemName=tamasfe.even-better-toml)

```bash
code --reuse-window \
     --install-extension ms-python.python \
     --install-extension ms-python.vscode-pylance \
     --install-extension ms-python.mypy-type-checker \
     --install-extension charliermarsh.ruff \
     --install-extension tamasfe.even-better-toml
```

For Rust development, the recommended extensions include:

- [Rust Analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer)
- [Even Better TOML](https://marketplace.visualstudio.com/items?itemName=tamasfe.even-better-toml)

```bash
code --reuse-window \
     --install-extension rust-lang.rust-analyzer \
     --install-extension tamasfe.even-better-toml
```

### Workspace Settings

SRB comes with [workspace settings](https://github.com/AndrejOrsula/space_robotics_bench/blob/main/.vscode/settings.json) that primarily configure Python paths for Isaac Sim/Lab modules. It does so with these assumptions (which you can adjust as needed):

- Isaac Sim is installed at `~/isaac-sim`
- Isaac Lab is installed at `../isaaclab`

## Using the Dev Container

For a pre-configured development environment, consider using [Dev Container](devcontainer.md), which automatically includes all necessary tools and extensions.
