# Rust Crate Checklist

## Project Setup

- [ ] `mtem create rust <name>` ran — `devenv.nix`, `devenv.yaml`, `devenv.lock` present
- [ ] `.envrc` created (`echo "use devenv" > .envrc && direnv allow`)
- [ ] `cargo init` or `cargo new` ran inside devenv shell
- [ ] `Cargo.toml` has `edition`, `rust-version`, `license`, `description`, `repository`, `documentation`

## Code Standards

- [ ] All public items have `///` doc comments
- [ ] Every public function has at least one `# Examples` doctest
- [ ] `# Errors` section present on all `Result`-returning functions
- [ ] Custom error type in `src/error.rs` using `thiserror`
- [ ] No `unwrap()` / `expect()` in library code
- [ ] Every `unsafe` block has a `// SAFETY:` comment

## Testing

- [ ] Unit tests in `#[cfg(test)]` blocks alongside the code
- [ ] Integration tests in `tests/`
- [ ] Doc tests pass (`cargo test --doc`)
- [ ] Coverage >90% (`cargo tarpaulin` or `llvm-cov`)

## Tooling

- [ ] `cargo fmt --check` exits 0
- [ ] `cargo clippy -- -D warnings` exits 0
- [ ] `cargo test` exits 0
- [ ] `cargo doc --no-deps` exits 0 with no warnings
- [ ] `cargo audit` exits 0

## Release

- [ ] `CHANGELOG.md` updated under a dated heading
- [ ] Version bumped in `Cargo.toml` (semver)
- [ ] `cargo publish --dry-run` succeeds
- [ ] Git tag created: `git tag -s v<version>`
- [ ] `cargo publish` ran
