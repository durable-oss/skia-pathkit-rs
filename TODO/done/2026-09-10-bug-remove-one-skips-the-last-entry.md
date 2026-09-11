# `SkIntersections::remove_one` does not remove the last entry, and loses coincidence flags

Found: 2026-09-10, by clippy once the orphaned modules compiled (item 01).
Fixed: `f149261`.

## Symptom

Two, neither of which looks like the same bug from the outside:

1. **`check_coincident` hung** (see
   `2026-09-10-bug-check-coincident-infinite-loop.md`). It asked for a removal,
   got none, and looked at the same entry again.
2. **Coincidence flags ended up on the wrong intersections** after any removal,
   silently. Nothing reported this; it would surface much later as a coincident
   run attached to the wrong span.

## Cause

`src/pathops/sk_intersections.rs`. C++ is:

```cpp
void SkIntersections::removeOne(int index) {
    int remaining = --fUsed - index;        // decrement FIRST
    if (remaining <= 0) {
        return;
    }
    memmove(...);
    int coBit = fIsCoincident[0] & (1 << index);
    fIsCoincident[0] -= ((fIsCoincident[0] >> 1) & ~((1 << index) - 1)) + coBit;
    fIsCoincident[1] -= ((fIsCoincident[1] >> 1) & ~((1 << index) - 1)) + coBit;
}
```

The port was:

```rust
let remaining = self.f_used as usize - index - 1;
if remaining <= 0 {
    return;                    // returns WITHOUT decrementing
}
for i in 0..remaining { ... }
self.f_used -= 1;              // only reached when something shifted
```

Two defects:

- **The count only dropped when something needed shifting.** Removing the last
  entry left `f_used` untouched, so the entry was still there. This is the
  direct cause of the hang.
- **`fIsCoincident` was never touched.** The bitmask is indexed in parallel with
  the entries, so after a removal every flag above `index` referred to the entry
  that used to be one slot higher.

## Fix

Decrement first, then early-return, then shift the coincidence bits down by one
and drop the bit belonging to the removed entry:

```rust
self.f_used -= 1;
let remaining = self.f_used as usize - index;
if remaining == 0 {
    return;
}
// ... shift entries ...
let keep_mask = !((1u16 << index) - 1);
for side in 0..2 {
    let co_bit = self.f_is_coincident[side] & (1 << index);
    self.f_is_coincident[side] = self.f_is_coincident[side]
        .wrapping_sub(((self.f_is_coincident[side] >> 1) & keep_mask) + co_bit);
}
```

## Why it went unnoticed

`SkIntersections.rs` was orphaned — present under `src/pathops/` but absent from
`mod.rs`, so never compiled. See `01-wire-orphaned-modules.md`.

Worth noting that the pre-existing `test_remove_one` passed both before and
after: it removes a middle entry from three, which is the one case the old code
handled. The failure was only at the boundary.

## Regression cover

In `sk_intersections.rs`:

- `remove_one_drops_the_last_entry` — removes the final entry, then the only
  remaining one, asserting the count falls each time.
- `remove_one_shifts_the_coincident_bits_down` — flags the last of three
  coincident, removes the first, and asserts the flag followed its entry down to
  index 1 rather than staying at index 2.
