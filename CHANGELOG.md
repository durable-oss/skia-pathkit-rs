# Changelog

## Unreleased

- Initial crate scaffold: ported from Google Skia's PathKit (`old/pathkit/`).
- Implemented: `core::Point`/`IPoint`, `core::Rect`, `core::Matrix`,
  `core::scalar` helpers, and the small enums in `core::types`
  (`FillType`, `Direction`, `Verb`).
- Scaffolded (public API defined, behavior stubbed with `todo!()` /
  `unimplemented!()`): `core::Path`, `core::PathBuilder`, `core::RRect`,
  `core::StrokeRec::apply_to_path`, `effects::PathEffect`, and the
  `pathops` boolean-operation module (`op`, `simplify`, `tight_bounds`,
  `as_winding`, `OpBuilder`).
- See `PORTING.md` for the full file-by-file status and suggested porting
  order for the remaining work.
