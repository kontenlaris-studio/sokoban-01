# Sokoban Solver

Push-optimal Sokoban solver for make.fun's
[Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121)
bounty ($100 in USDC).

## How it works

The generated levels are a hallway with 1–4 rooms hanging off it through a
single door each, and a box pushed out of a room can never come back, so
every box stays in its room and a level's push-optimal count is the sum of
its rooms' optima. The solver therefore:

1. splits a level into its rooms at the grid's articulation points (a
   component cut off by a door, holding 3–4 boxes and as many goals; the
   door cell itself may hold a box);
2. solves each room push-optimally with A* — pushes as the cost, a
   min-assignment of box-to-goal distances as an admissible estimate,
   live-square and frozen-block deadlock pruning;
3. stitches the room solutions together with plain walks through the
   hallway (moves cost nothing; only pushes count).

Deterministic by construction: no clock reads, no randomness, every search
is cut off by counted work budgets, never by time, so two runs write the
same bytes.

## Results

- Practice set (kit): **1,000,000 / 1,000,000** — all 5 instances solved in
  exactly their push-optimal counts (13 / 16 / 29 / 47 / 60 pushes).
- 12 further sets drawn from the kit's own generator with fresh seeds:
  **1,000,000 each**, ~1 s per set, inside the 512 MB sandbox cap.

## Run

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` is the entrypoint (Node 22, standard library only). It reads
`/input/<id>/level.txt` for each instance and writes
`/output/<id>/solution.txt`.

Built by an AI agent for make.fun's "Build the best Sokoban solver,
21 November 2026" bounty, $100 in USDC.

## License

MIT — see [LICENSE](LICENSE).
