/-
Why a Newton's cradle is hard for a simultaneous contact solver, and what it
would take to get it right.

The ideal behaviour -- the struck end ball stops, the far end ball leaves at the
incoming speed, the middle balls never move -- is the outcome of a chain of
SEQUENTIAL, ELASTIC, equal-mass collisions. Two facts decide whether an engine
can reproduce it:

  1. An equal-mass elastic collision swaps the two velocities. Chaining that
     down a line carries the incoming velocity to the far end and leaves every
     interior ball at rest: one-in-one-out, conserving momentum and energy.

  2. A solver that instead resolves a whole touching cluster at once, sharing
     the momentum across it (MuJoCo's convex, restitution-free contact solve is
     of this kind), conserves momentum but strictly loses kinetic energy when
     the balls are not already moving together. That lost energy is what spreads
     the row into a fan instead of ejecting one ball.

The honest answer: MuJoCo's default cannot produce ideal one-in-one-out -- it is
neither sequential nor elastic. Getting close means recovering BOTH properties:
a gap between balls makes contacts sequential rather than one cluster solve, and
a stiff, underdamped contact approximates elastic bounce.

Everything below is concrete over the rationals with `decide`, so it needs no
Mathlib and builds in seconds.
-/

namespace NewtonsCradle

abbrev V := Rat

def momentum2 (a b : V) : V := a + b
def ke2 (a b : V) : V := a * a + b * b

/-- Equal-mass elastic collision: the two velocities swap. -/
def elastic (a b : V) : V × V := (b, a)

/-- A cluster solve that shares the total momentum equally across the pair --
the shape of a restitution-free simultaneous contact resolve. -/
def share2 (a b : V) : V × V := ((a + b) / 2, (a + b) / 2)

/-! ## An equal-mass elastic strike: full transfer, no loss -/

theorem elastic_full_transfer : elastic (6 : V) 0 = (0, 6) := by decide

theorem elastic_keeps_momentum :
    momentum2 (elastic (6 : V) 0).1 (elastic (6 : V) 0).2 = momentum2 6 0 := by native_decide

theorem elastic_keeps_energy :
    ke2 (elastic (6 : V) 0).1 (elastic (6 : V) 0).2 = ke2 6 0 := by native_decide

/-! ## One-in-one-out down a line of five

Fold the swap down the row. From a single incoming ball the velocity walks to
the far end and every interior ball is left at rest. This holds for any incoming
speed -- it is definitional unfolding of the swap, not arithmetic. -/

abbrev Row := V × V × V × V × V

def walk (v : V) : Row :=
  let s1 := elastic v 0
  let s2 := elastic s1.2 0
  let s3 := elastic s2.2 0
  let s4 := elastic s3.2 0
  (s1.1, s2.1, s3.1, s4.1, s4.2)

theorem walk_is_one_in_one_out (v : V) : walk v = (0, 0, 0, 0, v) := by
  simp [walk, elastic]

/-! ## Why a simultaneous cluster solve cannot match it

For the two-ball strike (6, 0): the elastic result is (0, 6), energy 36. The
shared solve gives (3, 3): same momentum 6, but energy 18. Half the energy is
gone, and that loss is exactly what the real cradle does not incur. -/

theorem share_keeps_momentum :
    momentum2 (share2 (6 : V) 0).1 (share2 (6 : V) 0).2 = momentum2 6 0 := by native_decide

theorem share_halves_energy :
    ke2 (share2 (6 : V) 0).1 (share2 (6 : V) 0).2 = (18 : V)
    ∧ ke2 (6 : V) 0 = 36 := by native_decide

/-- The elastic strike keeps 18 more units of energy than the shared one, so the
two are genuinely different resolutions of the same momentum-conserving input.
That non-zero gap is what the tuning tries to close. -/
theorem elastic_beats_share :
    ke2 (elastic (6 : V) 0).1 (elastic (6 : V) 0).2
      - ke2 (share2 (6 : V) 0).1 (share2 (6 : V) 0).2 = 18 := by native_decide

/-! ## The conclusion the model licenses

`walk_is_one_in_one_out`: sequential elastic collisions reproduce one-in-one-out
exactly, for any incoming speed.

`share_halves_energy` / `elastic_beats_share`: a simultaneous equal-share resolve
conserves momentum but loses energy, so it cannot.

MuJoCo's contact solve is the second kind and carries no restitution, so it
cannot be ideal on its own. The model says what closing the gap needs: make
contacts SEQUENTIAL (a gap between balls, so the solver never resolves the whole
row at once) and ELASTIC (a stiff, underdamped contact approximating the swap).
That is exactly GAP > 0 and a stiff SOLREF in scripts/make_newtons_cradle.py,
which moves the sim from a fan toward one-in-one-out, bounded by how close
MuJoCo's dissipative contact gets to a true velocity swap.
-/

end NewtonsCradle
