# Graphical User Interface (GUI)

SRB comes with a minimal GUI application that can serve as a more approachable demonstration for non-developers. The GUI is written in Rust using the [egui](https://github.com/emilk/egui) framework, while the middleware between the GUI and the SRB framework is facilitated via the [r2r](https://github.com/sequenceplanner/r2r) ROS 2 bindings.

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/zijvZRoUGQ8?si=5jAhBuNbRowkcsEN&mute=1&autoplay=1&loop=1&playlist=zijvZRoUGQ8" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

At the moment, only the `srb teleop` workflow is supported by the GUI. However, it can be easily extended to support other workflows if there is interest.

## `srb gui` — Launch GUI

The `srb gui` command simplifies the launch of the GUI application via `cargo run`:

```
srb gui
```

> **Hint:** This command supports forwarding of all arguments following `--`.

### Example

Run the GUI with a release build:

```bash
srb gui -- --release
```
