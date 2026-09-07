//! Crate error type.

use thiserror::Error;

/// Errors returned by fallible `pathkit` operations.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum PathKitError {
    /// A boolean path operation ([`crate::pathops::op`]) could not produce
    /// a result, typically because the inputs contain a degenerate or
    /// self-intersecting configuration the solver could not resolve.
    #[error("path operation did not produce a result")]
    OperationFailed,

    /// [`crate::pathops::simplify`] could not reduce the path to
    /// non-overlapping contours.
    #[error("path simplification did not produce a result")]
    SimplifyFailed,

    /// A requested feature is scaffolded but not yet implemented in this
    /// port.
    #[error("not yet implemented: {0}")]
    Unimplemented(&'static str),

    /// Invalid arguments were passed to a function.
    #[error("invalid argument: {0}")]
    InvalidArgument(String),

    /// A conversion to or from another path representation is unsupported.
    #[error("unsupported conversion: {0}")]
    UnsupportedConversion(String),
}
