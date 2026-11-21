[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/krida268/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/krida268/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

# Room-by-Room Push-Optimal Sokoban Solver

A Sokoban solver for make.fun's sokoban-solver-01-run-20261121 bounty. It
exploits the structure of the generated levels: a one-square-wide hallway
where the player starts, plus one to four rooms off it. A box can never leave
its room in a valid solution, so a level's push optimum is the sum of its
rooms' optima — and each room is small enough to solve exactly.

## Approach

1. **Live squares.** A reverse pull BFS from every goal marks the squares
   from which a lone box can still reach a goal. A box anywhere else is dead.
2. **Room decomposition.** Cutting the non-live squares on the player's row
   splits the floor into components; each component holding boxes is a room
   with its own boxes and goals. If the shape is unexpected, the whole level
   falls back to a single subproblem.
3. **Rooms in sequence.** Rooms are solved one at a time on the real board.
   While a room is being solved, the other rooms' boxes (at their current,
   actual positions) act as walls. After each room, its moves are replayed on
   the real board to update the box and player positions.
4. **Push-level A\*.** Each room is solved with A\* over pushes, not steps:
   a state is the sorted box squares plus the canonical player square (the
   smallest square of the player's reachable region). The heuristic is a
   minimum-cost bipartite matching between boxes and goals, where each cost
   is the single-box push distance from a reverse BFS per goal — admissible,
   so the first solution found is push-optimal. States are pruned when a box
   lands on a non-live square or completes a frozen 2×2 block with a box off
   its goal. A greedy best-first search with its own node budget is the
   fallback if A\* ever exhausts its budget.
5. **Reconstruction.** Parent pointers store each push; walking moves between
   pushes are filled in with a BFS over the player's reachable floor and
   written lowercase, pushes uppercase.

Everything is deterministic: search budgets are counted in expanded nodes,
never in wall-clock time, and there is no clock or random input anywhere.

On the kit's practice set this solver scores the maximum: every instance is
solved in exactly P\* pushes (13 / 16 / 29 / 47 / 60).

## Run

```sh
MAKEFUN_INPUT_DIR=/input MAKEFUN_OUTPUT_DIR=/output node main.ts
```

The program reads each `<id>/level.txt` under the input directory (sorted)
and writes `<id>/solution.txt` under the output directory, creating the
per-instance folders itself. Node 22, no dependencies.

## Referral

make.fun referral code: enzt809f

## License

MIT — see LICENSE.
