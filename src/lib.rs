//! A Rust port of [Google Skia's PathKit][pathkit]: 2D path construction,
//! affine/perspective transforms, and boolean path operations
//! (union/intersect/difference/xor/simplify).
//!
//! [pathkit]: https://skia.org/docs/user/modules/pathkit/
//!
//! # Port status
//!
//! This crate is being ported incrementally from the original C++. See
//! `PORTING.md` in the repository root for a file-by-file status table.
//! Implemented so far: [`core::Point`]/[`core::IPoint`], [`core::Rect`],
//! [`core::Matrix`], and small standalone enums such as [`core::FillType`]
//! and [`core::Verb`]. Path construction ([`core::Path`],
//! [`core::PathBuilder`]) and the `pathops` boolean-operation engine are
//! scaffolded but not yet implemented — their public functions currently
//! panic with `todo!()` or return `Err`/`unimplemented!()`.
//!
//! # Examples
//!
//! ```
//! use pathkit::core::Rect;
//!
//! let r = Rect::from_ltrb(0.0, 0.0, 100.0, 50.0);
//! assert_eq!((r.center_x(), r.center_y()), (50.0, 25.0));
//! ```

#![warn(missing_docs)]

pub mod core;
pub mod effects;
pub mod error;
pub mod gpu;
#[cfg(feature = "skia-rs")]
pub mod interop;
pub mod pathops;

pub use error::PathKitError;
