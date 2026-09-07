//! Small memory-buffer utilities.
//!
//! Ported from `src/core/SkMalloc.h`; the malloc/realloc wrappers were
//! dropped since Rust's `Vec`/`Box` already provide safe allocation.

/// Zeroes every element of `buffer`.
pub fn sk_bzero<T: Default>(buffer: &mut [T]) {
    for elem in buffer.iter_mut() {
        *elem = T::default();
    }
}

/// Copies `min(dst.len(), src.len())` elements from `src` into `dst`.
///
/// Unlike `memcpy`, this never requires non-empty buffers, so it safely
/// handles empty slices.
pub fn sk_careful_memcpy<T: Copy>(dst: &mut [T], src: &[T]) {
    let copy_len = dst.len().min(src.len());
    if copy_len > 0 {
        dst[..copy_len].copy_from_slice(&src[..copy_len]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sk_bzero() {
        let mut buffer = [1u8, 2, 3, 4, 5];
        sk_bzero(&mut buffer);
        assert!(buffer.iter().all(|&x| x == 0));
    }

    #[test]
    fn test_sk_careful_memcpy() {
        let mut dst = [0u8; 5];
        let src = [1, 2, 3, 4, 5];
        sk_careful_memcpy(&mut dst, &src);
        assert_eq!(dst, src);
    }

    #[test]
    fn test_sk_careful_memcpy_truncate_src() {
        let mut dst = [0u8; 10];
        let src = [1, 2, 3];
        sk_careful_memcpy(&mut dst, &src);
        assert_eq!(dst[..3], src);
        assert!(dst[3..].iter().all(|&x| x == 0));
    }

    #[test]
    fn test_sk_careful_memcpy_empty() {
        let mut dst: [u8; 0] = [];
        let src: [u8; 0] = [];
        sk_careful_memcpy(&mut dst, &src);
    }
}
