# Testing

Automated testing is an essential part of the development process to ensure the correctness and reliability of the Space Robotics Bench.

## Scope

The SRB test suite focuses on integration tests of various workflows that are applied sequentially to all registered environments. As these tests require Isaac Sim to be running, NVIDIA GPU must be available on the system. For this reason, the test suite is not run automatically via CI/CD pipelines and must be executed manually.

## `srb test` — Run Tests

The `srb test` command simplifies running the test suites of SRB via `pytest` and/or `cargo test`.

```bash
srb test [options]
```

### Options

| Argument                   | Description             |  Default   |
| -------------------------- | ----------------------- | :--------: |
| `-l`/`--language`/`--lang` | Language suites to test | `[python]` |

> **Hint:** This command supports forwarding of all arguments following `--`.

### Examples

Forward `-vx` arguments to Python test suite (`pytest`):

```bash
srb test -- -vx
```

Run tests for both Python and Rust:

```bash
srb test --lang python rust
```
