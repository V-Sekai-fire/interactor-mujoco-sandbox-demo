/-
Guest memory in libriscv, and why snapshotting it and calling into it cheaply
were believed to be mutually exclusive.

Two shapes exist. A flat arena is one contiguous host buffer covering the
guest's address range. Paged memory is a map from page index to page data, and
neighbouring pages need not be adjacent in host memory.

Two operations are asked of it:

  * `memarray addr len` hands the host a pointer it can read as an array. It
    needs the whole span to be contiguous in host memory.
  * `serialize` captures the guest so it can be restored elsewhere.

libriscv today refuses `serialize` whenever the flat arena is in use, so the
arena is switched off to get snapshots -- and every multi-page `memarray` then
fails. This file states both properties and shows the refusal is not forced:
the flat case is the easy one to serialize, not the impossible one.
-/

namespace SandboxMem

/-- A page index and the bytes in it. -/
abbrev Page := Nat → UInt8

/-- Guest memory, in the two shapes libriscv actually builds. -/
inductive Mem where
  /-- One contiguous host buffer covering `[base, base + size)`. -/
  | flat (base size : Nat) (bytes : Nat → UInt8)
  /-- A map from page index to page, with no adjacency guarantee. -/
  | paged (pageSize : Nat) (pages : Nat → Option Page)

namespace Mem

/-- Whether an address holds a byte the guest can read. -/
def readable : Mem → Nat → Prop
  | flat base size _, a => base ≤ a ∧ a < base + size
  | paged ps pages, a => ps > 0 ∧ (pages (a / ps)).isSome

/-- The byte at an address, where one is readable. -/
def read : (m : Mem) → Nat → UInt8
  | flat base _ bytes, a => bytes (a - base)
  | paged ps pages, a => match pages (a / ps) with
    | some p => p (a % ps)
    | none => 0

/--
Whether the host may take one contiguous view of `[a, a + len)`.

The flat arena is one buffer, so any span inside it qualifies. Paged memory
only qualifies within a single page: two pages that are adjacent in guest
addresses are not thereby adjacent in host memory.
-/
def sequential : Mem → Nat → Nat → Prop
  | flat base size _, a, len => base ≤ a ∧ a + len ≤ base + size
  | paged ps _, a, len => ps > 0 ∧ a % ps + len ≤ ps

end Mem

/-- What a snapshot has to carry for the flat case. -/
structure Snapshot where
  base : Nat
  size : Nat
  bytes : Nat → UInt8

/-- libriscv as it stands: the flat arena is refused outright. -/
def serializeToday : Mem → Option Snapshot
  | .flat _ _ _ => none
  | .paged _ _ => none

/-- The arena is a contiguous buffer, so capturing it is a copy of that buffer. -/
def serializeFixed : Mem → Option Snapshot
  | .flat base size bytes => some ⟨base, size, bytes⟩
  | .paged _ _ => none

/-- Restoring rebuilds a flat arena from the captured buffer. -/
def deserialize (s : Snapshot) : Mem :=
  .flat s.base s.size s.bytes

/-! ## What the flat arena buys

Every span inside the arena is contiguous, so `memarray` never has to fail.
This is the property the 40-odd call sites in godot-sandbox rely on without
checking.
-/

theorem flat_span_sequential
    (base size : Nat) (bytes : Nat → UInt8) (a len : Nat)
    (h₁ : base ≤ a) (h₂ : a + len ≤ base + size) :
    (Mem.flat base size bytes).sequential a len :=
  ⟨h₁, h₂⟩

/-! ## What paged memory costs

A span that crosses a page boundary is not contiguous. This is the negative
control: without it the property above would be saying nothing, since a model
in which everything is sequential would satisfy it too.
-/

theorem paged_span_crossing_not_sequential
    (ps : Nat) (pages : Nat → Option Page) (a len : Nat)
    (hcross : ps < a % ps + len) :
    ¬ (Mem.paged ps pages).sequential a len := by
  intro h
  exact absurd h.2 (Nat.not_le.mpr hcross)

/-! ## The refusal is not forced

`serializeToday` never produces a snapshot of a flat arena, so a machine using
one cannot be migrated. `serializeFixed` always does, and restoring it returns
every readable byte unchanged.
-/

theorem today_refuses_flat (base size : Nat) (bytes : Nat → UInt8) :
    serializeToday (.flat base size bytes) = none := rfl

theorem fixed_accepts_flat (base size : Nat) (bytes : Nat → UInt8) :
    (serializeFixed (.flat base size bytes)).isSome := rfl

/-- A flat arena survives the round trip byte for byte. -/
theorem flat_round_trip
    (base size : Nat) (bytes : Nat → UInt8) (s : Snapshot)
    (hs : serializeFixed (.flat base size bytes) = some s) (a : Nat) :
    (deserialize s).read a = (Mem.flat base size bytes).read a := by
  simp [serializeFixed] at hs
  subst hs
  rfl

/-- Restoring preserves exactly which addresses are readable. -/
theorem flat_round_trip_readable
    (base size : Nat) (bytes : Nat → UInt8) (s : Snapshot)
    (hs : serializeFixed (.flat base size bytes) = some s) (a : Nat) :
    (deserialize s).readable a ↔ (Mem.flat base size bytes).readable a := by
  simp [serializeFixed] at hs
  subst hs
  exact Iff.rfl

/-! ## The conclusion the C++ has to match

Turning the arena off to gain snapshots trades a property that holds
everywhere for one that can be had directly. `serializeFixed` is the shape
`Memory::serialize_to` needs for the arena case: capture the buffer rather
than throw `FEATURE_DISABLED`.
-/

end SandboxMem
