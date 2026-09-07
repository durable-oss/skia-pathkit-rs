//! Arena allocator — low-level bump-allocator with destructor tracking.
//!
//! Ported from `old/pathkit/src/core/ArenaAlloc.h` /
//! `old/pathkit/src/core/ArenaAlloc.cpp`.
//!
//! # Layout
//!
//! Each heap-allocated block stores metadata before the bump region:
//!
//! ```text
//! [4 bytes: allocation size] [sizeof(ptr) prev dtor cursor]
//! [sizeof(FooterAction)+1 Footer(NextBlock)] [bump allocations…]
//! ```
//!
//! The 4-byte allocation-size prefix is Rust-specific (`dealloc` needs
//! the layout); the rest matches the C++ layout byte-for-byte so that
//! the destructor-walking logic (`NextBlock`, `SkipPod`, …) works
//! identically.

use std::alloc::{alloc, dealloc, Layout};
use std::mem;
use std::ptr;

// ---------------------------------------------------------------------------
// Internal types
// ---------------------------------------------------------------------------

/// Function signature stored in each footer: receives a pointer to the
/// byte *past* the footer and returns a pointer to the byte *past* the
/// preceding footer (or `null` to terminate the chain).
type FooterAction = unsafe fn(*mut u8) -> *mut u8;

/// The first 47 Fibonacci numbers. Fib(47) is the largest value < 2³².
/// Used by `FibBlockSizes` to grow block sizes.
const SK_FIBONACCI_47: [u32; 47] = [
    1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946,
    17711, 28657, 46368, 75025, 121393, 196418, 317811, 514229, 832040, 1346269, 2178309, 3524578,
    5702887, 9227465, 14930352, 24157817, 39088169, 63245986, 102334155, 165580141, 267914296,
    433494437, 701408733, 1134903170, 1836311903, 2971215073,
];

/// The number of bytes a `Footer` occupies: a function-pointer-sized
/// action plus a single padding byte.
const FOOTER_SIZE: usize = mem::size_of::<FooterAction>() + 1;

// ---------------------------------------------------------------------------
// Block-size progression (Fibonacci-like)
// ---------------------------------------------------------------------------

struct FibBlockSizes {
    index: u32,
    block_unit_size: u32,
}

impl FibBlockSizes {
    fn new(static_block_size: u32, first_allocation_size: u32) -> Self {
        let block_unit_size = if first_allocation_size > 0 {
            first_allocation_size
        } else if static_block_size > 0 {
            static_block_size
        } else {
            1024
        };
        debug_assert!(block_unit_size > 0);
        debug_assert!(block_unit_size < (1u32 << 26) - 1);
        FibBlockSizes {
            index: 0,
            block_unit_size,
        }
    }

    fn next_block_size(&mut self) -> u32 {
        let result = SK_FIBONACCI_47[self.index as usize] * self.block_unit_size;
        if (self.index as usize + 1) < SK_FIBONACCI_47.len()
            && SK_FIBONACCI_47[self.index as usize + 1] < u32::MAX / self.block_unit_size
        {
            self.index += 1;
        }
        result
    }
}

// ---------------------------------------------------------------------------
// Internal helper: convert `usize` → `u32` with a debug assertion
// ---------------------------------------------------------------------------

fn to_u32(v: usize) -> u32 {
    debug_assert!(v <= u32::MAX as usize);
    v as u32
}

// ---------------------------------------------------------------------------
// Static footer actions
// ---------------------------------------------------------------------------

/// Terminator: end of the footer chain. Returns null.
unsafe fn end_chain(_footer_end: *mut u8) -> *mut u8 {
    ptr::null_mut()
}

/// Reads a 4-byte skip count placed before the footer and moves the
/// cursor back past the POD region.
unsafe fn skip_pod(footer_end: *mut u8) -> *mut u8 {
    let obj_end = footer_end.sub(FOOTER_SIZE + mem::size_of::<i32>());
    let mut skip: i32 = 0;
    ptr::copy_nonoverlapping(
        obj_end,
        &mut skip as *mut i32 as *mut u8,
        mem::size_of::<i32>(),
    );
    obj_end.sub(skip as usize)
}

/// Reads the "previous block" pointer stored before the footer, runs
/// destructors on that block (recursing), then frees the current block.
unsafe fn next_block(footer_end: *mut u8) -> *mut u8 {
    let obj_end = footer_end.sub(mem::size_of::<*mut u8>() + FOOTER_SIZE);

    // The 4 bytes before obj_end hold the original allocation size.
    let alloc_start = obj_end.sub(4);
    let mut alloc_size: u32 = 0;
    ptr::copy_nonoverlapping(alloc_start, &mut alloc_size as *mut u32 as *mut u8, 4);

    // Read previous-block pointer.
    let mut next: *mut u8 = ptr::null_mut();
    ptr::copy_nonoverlapping(
        obj_end,
        &mut next as *mut *mut u8 as *mut u8,
        mem::size_of::<*mut u8>(),
    );

    run_dtors_on_block(next);

    // Free the entire block (including our 4-byte size prefix).
    let layout = Layout::from_size_align(alloc_size as usize, 1).unwrap();
    dealloc(alloc_start, layout);
    ptr::null_mut()
}

/// Walks the footer chain on a block, calling each footer action to
/// run destructors, until the chain terminator is reached.
unsafe fn run_dtors_on_block(mut footer_end: *mut u8) {
    while !footer_end.is_null() {
        let mut action: FooterAction = end_chain;
        let mut padding: u8 = 0;

        ptr::copy_nonoverlapping(
            footer_end.sub(FOOTER_SIZE),
            &mut action as *mut FooterAction as *mut u8,
            mem::size_of::<FooterAction>(),
        );
        ptr::copy_nonoverlapping(footer_end.sub(1), &mut padding as *mut u8, 1);

        footer_end = action(footer_end).sub(padding as usize);
    }
}

// ---------------------------------------------------------------------------
// ArenaAlloc — public interface
// ---------------------------------------------------------------------------

/// A bump allocator that tracks destructors for non-POD objects.
///
/// Objects can be allocated either from an optional user-provided block
/// or from heap-allocated blocks that grow with a Fibonacci progression.
/// When the arena is dropped, all registered destructors are called in
/// reverse order of allocation.
pub struct ArenaAlloc {
    dtor_cursor: *mut u8,
    cursor: *mut u8,
    end: *mut u8,
    fib_progression: FibBlockSizes,
}

impl ArenaAlloc {
    /// Creates an arena backed by an optional `block` of `block_size`
    /// bytes. When that block is exhausted, heap allocations start at
    /// `first_heap_allocation` bytes and grow via the Fibonacci
    /// progression.
    ///
    /// If `block` is null or `block_size` is zero, all allocations come
    /// from the heap.
    pub fn new(block: *mut u8, block_size: usize, first_heap_allocation: usize) -> Self {
        let mut alloc = ArenaAlloc {
            dtor_cursor: block,
            cursor: block,
            end: if block.is_null() {
                ptr::null_mut()
            } else {
                unsafe { block.add(block_size) }
            },
            fib_progression: FibBlockSizes::new(to_u32(block_size), to_u32(first_heap_allocation)),
        };

        if block_size < FOOTER_SIZE {
            alloc.end = ptr::null_mut();
            alloc.cursor = ptr::null_mut();
            alloc.dtor_cursor = ptr::null_mut();
        }

        if !alloc.cursor.is_null() {
            // SAFETY: cursor points inside the user-provided block or is
            // null (handled above). end_chain does not dereference its
            // argument.
            unsafe { alloc.install_footer(end_chain, 0) };
        }

        alloc
    }

    /// Creates an arena that allocates only from the heap, with
    /// `first_heap_allocation` as the initial block size.
    pub fn new_with_alloc(first_heap_allocation: usize) -> Self {
        ArenaAlloc::new(ptr::null_mut(), 0, first_heap_allocation)
    }

    // -----------------------------------------------------------------------
    // Public construction helpers
    // -----------------------------------------------------------------------

    /// Constructs an object of type `T` inside the arena, calling `ctor`
    /// with a pointer to freshly reserved storage.
    ///
    /// For trivially-destructible `T`, no footer is installed. For
    /// non-trivially-destructible `T`, a footer with `drop_in_place` is
    /// installed.
    ///
    /// Returns the value returned by `ctor` (typically a pointer).
    ///
    /// # Safety
    ///
    /// `ctor` must write a valid `T` into the given memory. The arena
    /// will later call `ptr::drop_in_place` on the object.
    pub unsafe fn make_with<T, R>(&mut self, ctor: impl FnOnce(*mut T) -> R) -> R {
        let size = to_u32(mem::size_of::<T>());
        let alignment = to_u32(mem::align_of::<T>());

        if mem::needs_drop::<T>() {
            // Non-trivial destructor: allocate with footer.
            let obj_start = self.alloc_object_with_footer(size + to_u32(FOOTER_SIZE), alignment);

            // Padding = offset of object from cursor after alignment.
            // Can never be UB because max value is alignof(T).
            let padding = to_u32(obj_start.offset_from(self.cursor) as usize);

            // Advance past the object to install footer at end.
            self.cursor = obj_start.add(size as usize);
            let action: FooterAction = destroy_object::<T>;
            self.install_footer(action, padding);

            ctor(obj_start as *mut T)
        } else {
            // Trivially destructible: just bump-allocate.
            let obj_start = self.alloc_object(size, alignment);
            self.cursor = obj_start.add(size as usize);
            ctor(obj_start as *mut T)
        }
    }

    /// Constructs a single `T` inside the arena using placement-new.
    ///
    /// # Safety
    ///
    /// The object's destructor must be called via `drop` on the arena
    /// (it will be when the arena is dropped).
    pub unsafe fn make<T>(&mut self, val: T) -> *mut T {
        self.make_with::<T, _>(|p| {
            ptr::write(p, val);
            p
        })
    }

    /// Constructs an array of `T` inside the arena with default
    /// initialization. For primitive `T`, elements are left
    /// uninitialized.
    ///
    /// # Safety
    ///
    /// The array elements' destructors (if any) will be called when the
    /// arena is dropped.
    pub unsafe fn make_array_default<T>(&mut self, count: usize) -> *mut T {
        assert!(to_u32(count) == count as u32);
        let count_u32 = to_u32(count);
        let _ = count_u32;

        let array_size = to_u32(count) * to_u32(mem::size_of::<T>());
        let alignment = to_u32(mem::align_of::<T>());

        let obj_start = if mem::needs_drop::<T>() {
            let overhead = to_u32(FOOTER_SIZE) + to_u32(mem::size_of::<u32>());
            let total_size = array_size + overhead;
            let start = self.alloc_object_with_footer(total_size, alignment);
            let padding = to_u32(start.offset_from(self.cursor) as usize);
            self.cursor = start.add(array_size as usize);
            self.install_raw(&count_u32);
            let action: FooterAction = destroy_array::<T>;
            self.install_footer(action, padding);
            start
        } else {
            let start = self.alloc_object(array_size, alignment);
            self.cursor = start.add(array_size as usize);
            start
        };

        obj_start as *mut T
    }

    // -----------------------------------------------------------------------
    // Low-level allocation
    // -----------------------------------------------------------------------

    /// Allocates `size` bytes with the given `alignment` for a
    /// trivially-destructible (POD) object. Does not install a footer.
    unsafe fn alloc_object(&mut self, size: u32, alignment: u32) -> *mut u8 {
        let mask = (alignment - 1) as usize;
        let cursor_addr = self.cursor as usize;

        let aligned_addr = cursor_addr.wrapping_add(mask) & !mask;
        let aligned_offset = aligned_addr - cursor_addr;
        let total_size = size as usize + aligned_offset;

        if total_size > (self.end as usize).wrapping_sub(cursor_addr) {
            self.ensure_space(size, alignment);
            let new_cursor_addr = self.cursor as usize;
            let new_aligned_addr = new_cursor_addr.wrapping_add(mask) & !mask;
            let new_aligned_offset = new_aligned_addr - new_cursor_addr;
            return self.cursor.add(new_aligned_offset);
        }

        self.cursor.add(aligned_offset)
    }

    /// Allocates `size_including_footer` bytes (which must include room
    /// for the footer) with the given `alignment`. Returns a pointer to
    /// the start of the user-accessible region.
    unsafe fn alloc_object_with_footer(
        &mut self,
        size_including_footer: u32,
        alignment: u32,
    ) -> *mut u8 {
        let mask = alignment as usize - 1;

        // We use a loop with goto in C++; translate to a loop in Rust.
        loop {
            let skip_overhead = if self.cursor != self.dtor_cursor {
                FOOTER_SIZE + mem::size_of::<u32>()
            } else {
                0
            };
            let total_size = size_including_footer as usize + skip_overhead;

            if self.cursor.is_null() {
                self.ensure_space(size_including_footer, alignment);
                continue;
            }

            let obj_start = (((self.cursor as usize)
                .wrapping_add(skip_overhead)
                .wrapping_add(mask))
                & !mask) as *mut u8;

            if total_size > (self.end as usize as isize).wrapping_sub(obj_start as isize) as usize {
                self.ensure_space(size_including_footer, alignment);
                continue;
            }

            debug_assert!(total_size <= (self.end as usize).wrapping_sub(obj_start as usize));

            // Install a skip footer if needed (terminating a run of POD data).
            if self.cursor != self.dtor_cursor {
                let skip = to_u32(self.cursor.offset_from(self.dtor_cursor) as usize);
                self.install_raw(&skip);
                self.install_footer(skip_pod, 0);
            }

            return obj_start;
        }
    }

    /// Ensures there is room for an allocation of `size` bytes at
    /// `alignment` by allocating a new heap block.
    unsafe fn ensure_space(&mut self, size: u32, alignment: u32) {
        let header_size = to_u32(FOOTER_SIZE) + to_u32(mem::size_of::<isize>());
        let max_size = u32::MAX;
        let overhead = header_size + to_u32(FOOTER_SIZE);
        debug_assert!(size <= max_size - overhead);
        let mut obj_size_and_overhead = size + overhead;

        let alignment_overhead = alignment - 1;
        debug_assert!(obj_size_and_overhead <= max_size - alignment_overhead);
        obj_size_and_overhead = obj_size_and_overhead.wrapping_add(alignment_overhead);

        let min_allocation = self.fib_progression.next_block_size();
        let mut allocation_size = obj_size_and_overhead.max(min_allocation);

        // Round up to a nice size.
        let mask = if allocation_size > 1 << 15 {
            (1 << 12) - 1
        } else {
            16 - 1
        };
        debug_assert!(allocation_size <= max_size - mask);
        allocation_size = (allocation_size + mask) & !mask;

        // Add room for the 4-byte size prefix we store before the block.
        let total_alloc = allocation_size as usize + 4;
        let layout = Layout::from_size_align(total_alloc, 1).unwrap();
        let new_block = alloc(layout);

        // Store the allocation size.
        ptr::write(new_block as *mut u32, allocation_size);

        let data_start = new_block.add(4); // skip our size prefix
        let previous_dtor = self.dtor_cursor;
        self.cursor = data_start;
        self.dtor_cursor = data_start;
        self.end = data_start.add(allocation_size as usize);
        self.install_raw(&previous_dtor);
        self.install_footer(next_block, 0);
    }

    // -----------------------------------------------------------------------
    // Footer management
    // -----------------------------------------------------------------------

    /// Appends a footer to the current cursor position and updates
    /// `dtor_cursor`.
    unsafe fn install_footer(&mut self, action: FooterAction, padding: u32) {
        debug_assert!(padding <= u8::MAX as u32);
        self.install_raw(&action);
        self.install_raw(&(padding as u8));
        self.dtor_cursor = self.cursor;
    }

    /// Copies `val` into the bump region at the current cursor position
    /// and advances the cursor.
    unsafe fn install_raw<T>(&mut self, val: &T) {
        ptr::copy_nonoverlapping(
            val as *const T as *const u8,
            self.cursor,
            mem::size_of::<T>(),
        );
        self.cursor = self.cursor.add(mem::size_of::<T>());
    }
}

impl Drop for ArenaAlloc {
    fn drop(&mut self) {
        unsafe {
            run_dtors_on_block(self.dtor_cursor);
        }
    }
}

// ---------------------------------------------------------------------------
// Non-portable helpers referenced by footer actions
// ---------------------------------------------------------------------------

/// Footer action for a single non-trivially-destructible object of type
/// `T`: calls `drop_in_place` and returns the start of the object
/// (which is also the end of the preceding footer, minus padding).
unsafe fn destroy_object<T>(obj_end: *mut u8) -> *mut u8 {
    let obj_start = obj_end.sub(mem::size_of::<T>() + FOOTER_SIZE);
    ptr::drop_in_place(obj_start as *mut T);
    obj_start
}

/// Footer action for an array of non-trivially-destructible objects:
/// reads the element count stored just before the footer, then calls
/// `drop_in_place` on each element from last to first.
unsafe fn destroy_array<T>(footer_end: *mut u8) -> *mut u8 {
    let obj_end = footer_end.sub(FOOTER_SIZE + mem::size_of::<u32>());
    let mut count: u32 = 0;
    ptr::copy_nonoverlapping(
        obj_end,
        &mut count as *mut u32 as *mut u8,
        mem::size_of::<u32>(),
    );
    let obj_start = obj_end.sub(count as usize * mem::size_of::<T>());
    let slice = ptr::slice_from_raw_parts_mut(obj_start as *mut T, count as usize);
    ptr::drop_in_place(slice);
    obj_start
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// A type with a non-trivial destructor that records its drops.
    #[derive(Debug)]
    struct DropCounter<'a> {
        counter: &'a mut i32,
    }

    impl<'a> Drop for DropCounter<'a> {
        fn drop(&mut self) {
            *self.counter += 1;
        }
    }

    // ------------------------------------------------------------------
    // FibBlockSizes
    // ------------------------------------------------------------------

    #[test]
    fn fib_block_sizes_starts_small() {
        let mut fib = FibBlockSizes::new(256, 0);
        // First block size = Fib(0) * blockUnitSize = 1 * 256 = 256
        let size = fib.next_block_size();
        assert_eq!(size, 256);
        // Second = Fib(1) * 256 = 1 * 256 = 256 (Fibonacci 1 == 1)
        let size2 = fib.next_block_size();
        assert_eq!(size2, 256);
        // Third = Fib(2) * 256 = 2 * 256 = 512
        let size3 = fib.next_block_size();
        assert_eq!(size3, 512);
    }

    #[test]
    fn fib_block_sizes_respects_unit_size() {
        let mut fib = FibBlockSizes::new(0, 1000);
        assert_eq!(fib.next_block_size(), 1000); // Fib(0)=1 * 1000
        assert_eq!(fib.next_block_size(), 1000); // Fib(1)=1 * 1000
        assert_eq!(fib.next_block_size(), 2000); // Fib(2)=2 * 1000
        assert_eq!(fib.next_block_size(), 3000); // Fib(3)=3 * 1000
    }

    #[test]
    fn fib_defaults_to_1024() {
        let mut fib = FibBlockSizes::new(0, 0);
        assert_eq!(fib.next_block_size(), 1024);
    }

    // ------------------------------------------------------------------
    // ArenaAlloc — trivially destructible (POD)
    // ------------------------------------------------------------------

    #[test]
    fn arena_alloc_i32() {
        let mut arena = ArenaAlloc::new_with_alloc(4096);
        unsafe {
            let p = arena.make(42i32);
            assert_eq!(*p, 42);
        }
    }

    #[test]
    fn arena_alloc_multiple_i32() {
        let mut arena = ArenaAlloc::new_with_alloc(4096);
        unsafe {
            let a = arena.make(10i32);
            let b = arena.make(20i32);
            let c = arena.make(30i32);
            assert_eq!(*a, 10);
            assert_eq!(*b, 20);
            assert_eq!(*c, 30);
            // Pointers should be contiguous for same-size POD.
            assert!((b as usize) > (a as usize));
            assert!((c as usize) > (b as usize));
        }
    }

    #[test]
    fn arena_alloc_f32() {
        let mut arena = ArenaAlloc::new_with_alloc(4096);
        unsafe {
            let p = arena.make(3.14f32);
            assert!((*p - 3.14).abs() < 1e-6);
        }
    }

    #[test]
    fn arena_alloc_struct() {
        #[derive(Debug, Clone, PartialEq)]
        struct Pod {
            x: f32,
            y: f32,
        }

        let mut arena = ArenaAlloc::new_with_alloc(4096);
        unsafe {
            let p = arena.make(Pod { x: 1.0, y: 2.0 });
            assert_eq!((*p).x, 1.0);
            assert_eq!((*p).y, 2.0);
        }
    }

    // ------------------------------------------------------------------
    // ArenaAlloc — non-trivially destructible
    // ------------------------------------------------------------------

    #[test]
    fn arena_alloc_dropped_object() {
        let mut count = 0i32;
        let mut arena = ArenaAlloc::new_with_alloc(4096);
        unsafe {
            let _p = arena.make(DropCounter {
                counter: &mut count,
            });
            // Object is live inside arena; drop hasn't happened yet.
        }
        // Explicitly drop the arena so its destructor runs before we
        // inspect `count` (it otherwise wouldn't run until the end of
        // this function, i.e. after the assertion below).
        drop(arena);
        assert_eq!(count, 1);
    }

    #[test]
    fn arena_alloc_multiple_dropped_objects() {
        let mut count1 = 0i32;
        let mut count2 = 0i32;
        let mut arena = ArenaAlloc::new_with_alloc(4096);
        unsafe {
            arena.make(DropCounter {
                counter: &mut count1,
            });
            arena.make(DropCounter {
                counter: &mut count2,
            });
        }
        drop(arena);
        assert_eq!(count1, 1);
        assert_eq!(count2, 1);
    }

    #[test]
    fn arena_alloc_mixed_pod_and_dropped() {
        let mut count = 0i32;
        let mut arena = ArenaAlloc::new_with_alloc(4096);
        unsafe {
            arena.make(100i32); // POD
            arena.make(DropCounter {
                counter: &mut count,
            }); // non-POD
            arena.make(3.14f32); // POD
        }
        drop(arena);
        assert_eq!(count, 1);
    }

    // ------------------------------------------------------------------
    // ArenaAlloc — arrays
    // ------------------------------------------------------------------

    #[test]
    fn arena_alloc_pod_array() {
        let mut arena = ArenaAlloc::new_with_alloc(4096);
        unsafe {
            // make_array_default for POD leaves memory uninitialized;
            // we just check it doesn't crash and returns a valid pointer.
            let arr = arena.make_array_default::<u8>(100);
            assert!(!arr.is_null());
        }
    }

    // ------------------------------------------------------------------
    // ArenaAlloc — user-provided block
    // ------------------------------------------------------------------

    #[test]
    fn arena_uses_static_block() {
        let mut block = [0u8; 256];
        let mut arena = ArenaAlloc::new(block.as_mut_ptr(), 256, 0);
        unsafe {
            let p = arena.make(42i32);
            assert_eq!(*p, 42);
            // The object should be inside the static block.
            let block_range = block.as_ptr() as usize..block.as_ptr() as usize + block.len();
            assert!(block_range.contains(&(p as usize)));
        }
    }

    #[test]
    fn arena_falls_back_to_heap_when_static_exhausted() {
        let mut block = [0u8; 32]; // very small static block
        let mut arena = ArenaAlloc::new(block.as_mut_ptr(), 32, 256);
        unsafe {
            // Allocate many bytes that won't fit in 32 bytes.
            let vals: Vec<*mut u8> = (0..100).map(|_| arena.make(0u8)).collect();
            // All pointers must be valid (non-null, readable).
            for &v in &vals {
                assert_eq!(*v, 0);
            }
        }
    }

    #[test]
    fn arena_with_null_block_still_works() {
        let mut arena = ArenaAlloc::new(ptr::null_mut(), 0, 4096);
        unsafe {
            let p = arena.make(42i32);
            assert_eq!(*p, 42);
        }
    }

    // ------------------------------------------------------------------
    // Drop ordering: objects are destroyed in reverse order
    // ------------------------------------------------------------------

    #[test]
    fn arena_drops_in_reverse_order() {
        let mut drop_order = Vec::new();
        struct OrderCheck<'a> {
            id: i32,
            order: &'a mut Vec<i32>,
        }
        impl Drop for OrderCheck<'_> {
            fn drop(&mut self) {
                self.order.push(self.id);
            }
        }

        let mut arena = ArenaAlloc::new_with_alloc(4096);
        unsafe {
            arena.make(OrderCheck {
                id: 1,
                order: &mut drop_order,
            });
            arena.make(OrderCheck {
                id: 2,
                order: &mut drop_order,
            });
            arena.make(OrderCheck {
                id: 3,
                order: &mut drop_order,
            });
        }
        drop(arena);
        // Destructors run in reverse: 3, 2, 1
        assert_eq!(drop_order, vec![3, 2, 1]);
    }
}
