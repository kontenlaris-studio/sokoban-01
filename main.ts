// Sokoban solver: room-decomposed push-optimal A*.
//
// The generated levels are a hallway with 1-4 rooms hanging off it through a
// single door each, and a box pushed out of a room can never come back, so
// every box stays in its room and a level's push-optimal count is the sum of
// its rooms' optima. The solver therefore splits a level into its rooms at
// the grid's articulation points, solves each room push-optimally with A*
// (pushes as the cost, a min-assignment of box-to-goal distances as an
// admissible estimate, live-square and frozen-block deadlock pruning), then
// stitches the room solutions together with plain walks through the hallway.
// Moves cost nothing; only pushes count, so any valid walk between rooms
// is fine.
//
// Deterministic by construction: no clock reads, no randomness, every
// search is cut off by counted work budgets (expansions, nodes), never by
// time, so two runs write the same bytes. Node 22, standard library only.
//
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

/** [dx, dy, move letter] for up, down, left, right. */
const DIRS: ReadonlyArray<readonly [number, number, string]> = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];

/** Exact A* expansion budget per room attempt; the weighted fallback gets its own. */
const ROOM_BUDGET = 400_000;
/** Most search nodes one room attempt may create (memory bound). */
const NODE_CAP = 1_200_000;

interface Level {
  w: number;
  h: number;
  /** wall[i] true when cell i is a wall (or outside the level). */
  wall: boolean[];
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall: boolean[] = new Array(w * h).fill(true);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = false;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

/** The four neighbours of a cell that are not walls. */
function neighbours(lv: Level, i: number): number[] {
  const out: number[] = [];
  const x = i % lv.w;
  const y = Math.floor(i / lv.w);
  if (y > 0 && !lv.wall[i - lv.w]) out.push(i - lv.w);
  if (y + 1 < lv.h && !lv.wall[i + lv.w]) out.push(i + lv.w);
  if (x > 0 && !lv.wall[i - 1]) out.push(i - 1);
  if (x + 1 < lv.w && !lv.wall[i + 1]) out.push(i + 1);
  return out;
}

/** Tarjan articulation points of the floor graph (cells whose removal splits it). */
function articulationPoints(lv: Level): Set<number> {
  const disc = new Map<number, number>();
  const low = new Map<number, number>();
  const parent = new Map<number, number>();
  const ap = new Set<number>();
  let tick = 0;
  const dfs = (u: number): void => {
    disc.set(u, tick);
    low.set(u, tick);
    tick++;
    let children = 0;
    for (const v of neighbours(lv, u)) {
      if (!disc.has(v)) {
        parent.set(v, u);
        children++;
        dfs(v);
        low.set(u, Math.min(low.get(u)!, low.get(v)!));
        if (!parent.has(u) && children > 1) ap.add(u);
        if (parent.has(u) && low.get(v)! >= disc.get(u)!) ap.add(u);
      } else if (v !== parent.get(u)) {
        low.set(u, Math.min(low.get(u)!, disc.get(v)!));
      }
    }
  };
  for (let i = 0; i < lv.w * lv.h; i++) {
    if (!lv.wall[i] && !disc.has(i)) dfs(i);
  }
  return ap;
}

/** Connected components of the floor graph after removing one cell. */
function componentsWithout(lv: Level, removed: number): Set<number>[] {
  const seen = new Set<number>();
  const comps: Set<number>[] = [];
  for (let i = 0; i < lv.w * lv.h; i++) {
    if (lv.wall[i] || i === removed || seen.has(i)) continue;
    const comp = new Set<number>();
    const queue = [i];
    seen.add(i);
    while (queue.length > 0) {
      const u = queue.pop()!;
      comp.add(u);
      for (const v of neighbours(lv, u)) {
        if (v !== removed && !seen.has(v)) {
          seen.add(v);
          queue.push(v);
        }
      }
    }
    comps.push(comp);
  }
  return comps;
}

interface RoomCandidate {
  /** Floor cells a box may ever stand on: the component plus its door cell. */
  cells: Set<number>;
  /** Floor cells the player may walk on: cells plus the hallway approach. */
  walk: Set<number>;
  boxes: number[];
  goals: number[];
  /** The hallway cell the player stands on when the room solve begins. */
  approach: number;
}

/**
 * Split the level into rooms. A room is a component cut off by an
 * articulation point (its door) holding 3-4 boxes and as many goals; the
 * door cell itself may hold a box, so it counts as part of the room. The
 * same room is found through several doors, so candidates are grouped by
 * their box set and tried smallest first. A group whose boxes are a strict
 * subset of another group's is an artefact of cutting inside a room (the
 * cut strands one box outside) and is dropped. Returns null when the boxes
 * and goals do not partition cleanly into rooms.
 */
function decompose(lv: Level): Map<string, RoomCandidate[]> | null {
  const groups = new Map<string, RoomCandidate[]>();
  const claimedBoxes = new Set<number>();
  const claimedGoals = new Set<number>();
  for (const d of [...articulationPoints(lv)].sort((a, b) => a - b)) {
    for (const comp of componentsWithout(lv, d)) {
      if (comp.has(lv.player)) continue;
      const cells = new Set(comp);
      cells.add(d);
      const boxes = lv.boxes.filter((b) => cells.has(b));
      const goals = lv.goals.filter((g) => cells.has(g));
      if (boxes.length < 3 || boxes.length > 4) continue;
      if (boxes.length !== goals.length) continue;
      const approach: number[] = neighbours(lv, d)
        .filter((n) => !comp.has(n) && n !== d)
        .sort((a, b) => a - b);
      if (!boxes.includes(d)) approach.push(d);
      approach.sort((a, b) => a - b);
      if (approach.length === 0) continue;
      const walk = new Set(cells);
      for (const a of approach) walk.add(a);
      const cand: RoomCandidate = { cells, walk, boxes, goals, approach: approach[0]! };
      const key = boxes.join(',');
      const list = groups.get(key) ?? [];
      list.push(cand);
      groups.set(key, list);
    }
  }
  for (const list of groups.values()) {
    list.sort((a, b) => a.cells.size - b.cells.size || a.approach - b.approach);
  }
  const boxSets = new Map<string, Set<number>>();
  for (const key of groups.keys()) boxSets.set(key, new Set(key.split(',').map(Number)));
  for (const key of [...groups.keys()]) {
    const a = boxSets.get(key)!;
    for (const other of groups.keys()) {
      if (other === key) continue;
      const b = boxSets.get(other)!;
      if (a.size < b.size && [...a].every((x) => b.has(x))) {
        groups.delete(key);
        break;
      }
    }
  }
  const accepted = new Map<string, RoomCandidate[]>();
  for (const key of [...groups.keys()].sort()) {
    const list = groups.get(key)!;
    const boxes = key.split(',').map(Number);
    if (boxes.some((b) => claimedBoxes.has(b))) continue;
    const goals = list[0]!.goals;
    if (goals.some((g) => claimedGoals.has(g))) continue;
    for (const b of boxes) claimedBoxes.add(b);
    for (const g of goals) claimedGoals.add(g);
    accepted.set(key, list);
  }
  if (claimedBoxes.size !== lv.boxes.length) return null;
  if (claimedGoals.size !== lv.goals.length) return null;
  return accepted;
}

/** Breadth-first distances from one cell over a floor set (boxes ignored). */
function bfsDistances(lv: Level, from: number, floor: Set<number>): Map<number, number> {
  const dist = new Map<number, number>([[from, 0]]);
  const queue = [from];
  for (let qi = 0; qi < queue.length; qi++) {
    const u = queue[qi]!;
    for (const v of neighbours(lv, u)) {
      if (floor.has(v) && !dist.has(v)) {
        dist.set(v, dist.get(u)! + 1);
        queue.push(v);
      }
    }
  }
  return dist;
}

/** The cells the player can reach without moving a box, from a start cell. */
function playerRegion(lv: Level, start: number, boxes: Set<number>, walk: Set<number>): Set<number> {
  const seen = new Set<number>([start]);
  const queue = [start];
  for (let qi = 0; qi < queue.length; qi++) {
    const u = queue[qi]!;
    for (const v of neighbours(lv, u)) {
      if (walk.has(v) && !boxes.has(v) && !seen.has(v)) {
        seen.add(v);
        queue.push(v);
      }
    }
  }
  return seen;
}

/** A push: the box cell before the push and the direction index into DIRS. */
type Push = readonly [number, number];

interface RoomSolution {
  pushes: Push[];
  expanded: number;
}

/**
 * Push-optimal A* over one room (weight 1), or a weighted A* (weight > 1)
 * that still solves the room when the exact search runs out of budget.
 * A state is the sorted box cells plus the player's reachable region
 * (canonicalised to its smallest cell); each step pushes one box one cell.
 * The estimate is the cheapest assignment of boxes to goals by
 * box-ignoring distance (or, past 6 boxes, the sum of each box's nearest
 * goal distance), which no solution can beat. Two deadlocks are pruned,
 * and neither prunes a solvable state: a box on a non-live square (no
 * push sequence, other boxes ignored, brings it to a goal) and a box in
 * a frozen 2x2 block of walls and boxes with a box off its goal. Nodes
 * live in typed arrays behind a packed numeric key, so a search that
 * runs to its budget stays far inside the sandbox memory.
 */
function solveRoom(
  lv: Level,
  room: RoomCandidate,
  weight: number,
  budget: number,
): RoomSolution | null {
  const goals = room.goals;
  const goalSet = new Set(goals);
  const base = lv.w * lv.h;
  // Live squares: seeded at the goals, a square is live when a box on it
  // can be pushed (other boxes ignored) onto a live square.
  const live = new Set<number>(goals);
  const liveQueue = [...goals];
  for (let qi = 0; qi < liveQueue.length; qi++) {
    const x = liveQueue[qi]!;
    for (let di = 0; di < DIRS.length; di++) {
      const [dx, dy] = DIRS[di]!;
      const y = x - dy * lv.w - dx;
      const behind = y - dy * lv.w - dx;
      if (!room.cells.has(y) || !room.cells.has(behind) || live.has(y)) continue;
      live.add(y);
      liveQueue.push(y);
    }
  }
  if (room.boxes.some((b) => !live.has(b))) return null;
  // Distances between every pair of box cells, boxes ignored.
  const distFrom = new Map<number, Map<number, number>>();
  for (const c of room.cells) distFrom.set(c, bfsDistances(lv, c, room.cells));
  // All permutations of the goal indices, for the assignment estimate
  // (only built for small rooms; past 6 boxes the estimate below falls
  // back to the sum of nearest-goal distances, which stays admissible).
  const perms: number[][] = [];
  if (goals.length <= 6) {
    const gen = (prefix: number[], rest: number[]): void => {
      if (rest.length === 0) {
        perms.push(prefix);
        return;
      }
      for (let i = 0; i < rest.length; i++) {
        gen([...prefix, rest[i]!], [...rest.slice(0, i), ...rest.slice(i + 1)]);
      }
    };
    gen([], goals.map((_, i) => i));
  }
  const estimate = (bs: number[]): number => {
    if (perms.length > 0) {
      let best = Number.POSITIVE_INFINITY;
      for (const p of perms) {
        let s = 0;
        for (let i = 0; i < bs.length; i++) {
          const d = distFrom.get(bs[i]!)!.get(goals[p[i]!]!);
          if (d === undefined) {
            s = Number.POSITIVE_INFINITY;
            break;
          }
          s += d;
        }
        if (s < best) best = s;
      }
      return best;
    }
    let s = 0;
    for (const b of bs) {
      let m = Number.POSITIVE_INFINITY;
      for (const g of goals) {
        const d = distFrom.get(b)!.get(g);
        if (d !== undefined && d < m) m = d;
      }
      if (m === Number.POSITIVE_INFINITY) return m;
      s += m;
    }
    return s;
  };
  const pack = (bs: number[]): number => {
    let k = 0;
    for (const b of bs) k = k * base + b;
    return k;
  };
  const unpack = (k: number, n: number): number[] => {
    const bs = new Array<number>(n);
    for (let i = n - 1; i >= 0; i--) {
      bs[i] = k % base;
      k = Math.floor(k / base);
    }
    return bs;
  };
  const keyOf = (packed: number, rep: number): number => packed * base + rep;
  // Whether the box just pushed to `t` sits in a frozen 2x2 block.
  const frozen = (t: number, boxSet: Set<number>): boolean => {
    const x = t % lv.w;
    const y = Math.floor(t / lv.w);
    for (let oy = 0; oy <= 1; oy++) {
      for (let ox = 0; ox <= 1; ox++) {
        const ax = x - ox;
        const ay = y - oy;
        if (ax < 0 || ay < 0 || ax + 1 >= lv.w || ay + 1 >= lv.h) continue;
        const a = ay * lv.w + ax;
        const cells = [a, a + 1, a + lv.w, a + lv.w + 1];
        let allBlocked = true;
        let offGoal = false;
        for (const c of cells) {
          const blocked = !room.cells.has(c) || boxSet.has(c);
          if (!blocked) {
            allBlocked = false;
            break;
          }
          if (boxSet.has(c) && !goalSet.has(c)) offGoal = true;
        }
        if (allBlocked && offGoal) return true;
      }
    }
    return false;
  };
  // Node store: packed boxes, player cell, parent node, push that made it,
  // and the f/g scores plus insertion order for the heap. Typed arrays so
  // a search that runs to its node cap stays far inside sandbox memory.
  let cap = 1024;
  let packedBoxes = new Float64Array(cap);
  let nodePlayer = new Int32Array(cap);
  let nodeParent = new Int32Array(cap);
  let nodePushBox = new Int32Array(cap);
  let nodePushDir = new Int32Array(cap);
  let nodeF = new Float64Array(cap);
  let nodeG = new Int32Array(cap);
  let nodeOrder = new Int32Array(cap);
  let nodeCount = 0;
  const addNode = (
    packed: number,
    player: number,
    parent: number,
    pushBox: number,
    pushDir: number,
    f: number,
    g: number,
    order: number,
  ): number => {
    if (nodeCount === cap) {
      cap *= 2;
      const growF64 = (a: Float64Array): Float64Array => {
        const b = new Float64Array(cap);
        b.set(a);
        return b;
      };
      const growI32 = (a: Int32Array): Int32Array => {
        const b = new Int32Array(cap);
        b.set(a);
        return b;
      };
      packedBoxes = growF64(packedBoxes);
      nodePlayer = growI32(nodePlayer);
      nodeParent = growI32(nodeParent);
      nodePushBox = growI32(nodePushBox);
      nodePushDir = growI32(nodePushDir);
      nodeF = growF64(nodeF);
      nodeG = growI32(nodeG);
      nodeOrder = growI32(nodeOrder);
    }
    const t = nodeCount++;
    packedBoxes[t] = packed;
    nodePlayer[t] = player;
    nodeParent[t] = parent;
    nodePushBox[t] = pushBox;
    nodePushDir[t] = pushDir;
    nodeF[t] = f;
    nodeG[t] = g;
    nodeOrder[t] = order;
    return t;
  };
  // Binary heap of node indices, ordered by (f, g, insertion order).
  const heap: number[] = [];
  let heapOrder = 0;
  const heapLess = (a: number, b: number): boolean => {
    if (nodeF[a] !== nodeF[b]) return nodeF[a]! < nodeF[b]!;
    if (nodeG[a] !== nodeG[b]) return nodeG[a]! < nodeG[b]!;
    return nodeOrder[a]! < nodeOrder[b]!;
  };
  const heapPush = (t: number): void => {
    let i = heap.length;
    heap.push(t);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && heapLess(heap[c + 1]!, heap[c]!)) c++;
        if (!heapLess(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };
  const bestG = new Map<number, number>();
  const startBoxes = [...room.boxes].sort((a, b) => a - b);
  const startPacked = pack(startBoxes);
  const startRegion = playerRegion(lv, room.approach, new Set(startBoxes), room.walk);
  const startKey = keyOf(startPacked, Math.min(...startRegion));
  bestG.set(startKey, 0);
  const startNode = addNode(startPacked, room.approach, -1, -1, -1, estimate(startBoxes) * weight, 0, heapOrder++);
  heapPush(startNode);
  let expanded = 0;
  while (heap.length > 0) {
    const s = heapPop();
    const g = nodeG[s]!;
    const bs = unpack(packedBoxes[s]!, room.boxes.length);
    const region = playerRegion(lv, nodePlayer[s]!, new Set(bs), room.walk);
    const rep = Math.min(...region);
    const key = keyOf(packedBoxes[s]!, rep);
    if (bestG.get(key) !== g) continue;
    if (bs.every((b) => goalSet.has(b))) {
      const pushes: Push[] = [];
      for (let t = s; nodeParent[t]! >= 0; t = nodeParent[t]!) {
        pushes.push([nodePushBox[t]!, nodePushDir[t]!]);
      }
      pushes.reverse();
      return { pushes, expanded };
    }
    if (expanded >= budget) return null;
    expanded++;
    const boxSet = new Set(bs);
    for (let bi = 0; bi < bs.length; bi++) {
      const b = bs[bi]!;
      for (let di = 0; di < DIRS.length; di++) {
        const [dx, dy] = DIRS[di]!;
        const tgt = b + dy * lv.w + dx;
        if (!room.cells.has(tgt) || boxSet.has(tgt)) continue;
        const behind = b - dy * lv.w - dx;
        if (!region.has(behind)) continue;
        if (!live.has(tgt)) continue;
        const nbs = [...bs];
        nbs[bi] = tgt;
        nbs.sort((a, b2) => a - b2);
        const nBoxSet = new Set(nbs);
        if (frozen(tgt, nBoxSet)) continue;
        const nRegion = playerRegion(lv, b, nBoxSet, room.walk);
        const nRep = Math.min(...nRegion);
        const nPacked = pack(nbs);
        const nKey = keyOf(nPacked, nRep);
        const ng = g + 1;
        // Keep only the cheapest known path to each state.
        if (bestG.has(nKey) && bestG.get(nKey)! <= ng) continue;
        if (nodeCount >= NODE_CAP) return null;
        bestG.set(nKey, ng);
        const t = addNode(nPacked, b, s, b, di, ng + estimate(nbs) * weight, ng, heapOrder++);
        heapPush(t);
      }
    }
  }
  return null;
}

/** Shortest player walk (boxes as obstacles) between two cells, as moves. */
function walkPath(lv: Level, from: number, to: number, boxes: Set<number>, floor: Set<number>): string | null {
  if (from === to) return '';
  const prev = new Map<number, number>();
  const step = new Map<number, string>();
  const queue = [from];
  prev.set(from, -1);
  for (let qi = 0; qi < queue.length; qi++) {
    const u = queue[qi]!;
    if (u === to) break;
    for (let di = 0; di < DIRS.length; di++) {
      const [dx, dy, letter] = DIRS[di]!;
      const v = u + dy * lv.w + dx;
      if (!floor.has(v) || boxes.has(v) || prev.has(v)) continue;
      prev.set(v, u);
      step.set(v, letter);
      queue.push(v);
    }
  }
  if (!prev.has(to)) return null;
  let moves = '';
  for (let u = to; prev.get(u)! !== -1; u = prev.get(u)!) moves = step.get(u)! + moves;
  return moves;
}

/**
 * Solve one level. Rooms are solved independently (their push counts add
 * up to the level optimum), then stitched in nearest-door-first order.
 * Any failure leaves an empty solution: valid format, no points, but the
 * entry still answers the instance. The whole-level fallback only runs
 * for small levels; on a big one it would cost more than it can ever win.
 */
function solve(text: string): string {
  const lv = parseLevel(text);
  if (lv.boxes.length === 0) return '';
  const floor = new Set<number>();
  for (let i = 0; i < lv.w * lv.h; i++) if (!lv.wall[i]) floor.add(i);
  const groups = decompose(lv);
  const attempts: RoomCandidate[] = [];
  if (groups !== null) {
    for (const key of [...groups.keys()].sort()) {
      const list = groups.get(key)!;
      let solved: RoomSolution | null = null;
      let chosen: RoomCandidate | null = null;
      for (const cand of list) {
        solved = solveRoom(lv, cand, 1, ROOM_BUDGET);
        if (solved !== null) {
          chosen = cand;
          break;
        }
      }
      if (solved === null) {
        solved = solveRoom(lv, list[0]!, 2, ROOM_BUDGET);
        chosen = list[0]!;
      }
      if (solved === null || chosen === null) return '';
      attempts.push(chosen);
      (chosen as RoomCandidate & { solution?: RoomSolution }).solution = solved;
    }
  } else if (lv.boxes.length <= 6) {
    // No clean split: fall back to solving the whole level as one room.
    const whole: RoomCandidate = {
      cells: floor,
      walk: floor,
      boxes: lv.boxes,
      goals: lv.goals,
      approach: lv.player,
    };
    const solved = solveRoom(lv, whole, 1, ROOM_BUDGET) ?? solveRoom(lv, whole, 2, ROOM_BUDGET);
    if (solved === null) return '';
    (whole as RoomCandidate & { solution?: RoomSolution }).solution = solved;
    attempts.push(whole);
  } else {
    return '';
  }
  type SolvedRoom = RoomCandidate & { solution: RoomSolution };
  const solvedRooms = attempts as SolvedRoom[];
  let moves = '';
  let cur = lv.player;
  const curBoxes = new Set(lv.boxes);
  const remaining = [...solvedRooms];
  while (remaining.length > 0) {
    let bestIdx = -1;
    let bestWalk: string | null = null;
    for (let i = 0; i < remaining.length; i++) {
      const w = walkPath(lv, cur, remaining[i]!.approach, curBoxes, floor);
      if (w !== null && (bestWalk === null || w.length < bestWalk.length)) {
        bestWalk = w;
        bestIdx = i;
      }
    }
    if (bestIdx < 0 || bestWalk === null) return '';
    const room = remaining.splice(bestIdx, 1)[0]!;
    moves += bestWalk;
    cur = room.approach;
    for (const [box, di] of room.solution.pushes) {
      const [dx, dy, letter] = DIRS[di]!;
      const behind = box - dy * lv.w - dx;
      const w = walkPath(lv, cur, behind, curBoxes, floor);
      if (w === null) return '';
      moves += w;
      moves += letter;
      curBoxes.delete(box);
      curBoxes.add(box + dy * lv.w + dx);
      cur = box;
    }
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
