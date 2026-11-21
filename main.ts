// Sokoban solver: dekomposisi ruangan + A* atas dorongan dengan heuristik
// pencocokan bipartit atas jarak dorongan satu-kotak. Deterministik penuh:
// semua batas pencarian dalam jumlah ekspansi node, tanpa jam dinding.
//
// Ide besarnya: level generator terdiri dari lorong tempat pemain mulai dan
// ruangan-ruangan yang masing-masing tertutup untuk kotaknya sendiri, jadi
// optimum dorongan level adalah jumlah optimum tiap ruangan. Ruangan
// diselesaikan satu per satu di papan nyata; kotak ruangan lain diperlakukan
// sebagai dinding selama ruangan itu dikerjakan.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const IN_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const ASTAR_BUDGET = 1500000; // ekspansi per submasalah
const GREEDY_BUDGET = 1500000; // ekspansi fallback rakus per submasalah

// Arah tetap u, d, l, r — urutan ini bagian dari determinisme.
const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

// ---------- parsing (semantik sama dengan checker) ----------
function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const size = w * h;
  const wall = new Uint8Array(size).fill(1);
  const goal = new Uint8Array(size);
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, size, wall, goal, boxes, player };
}

// ---------- jarak dorongan satu kotak (BFS tarik mundur dari satu gol) ----------
// dist[c] = dorongan minimum membawa satu kotak dari c ke gol ini, kotak lain
// diabaikan; -1 bila tak terjangkau. Mundur: kotak di y berasal dari x = y - d
// hanya bila petak berdiri x - d juga lantai.
function pullDistances(wall, w, size, goalCell) {
  const dist = new Int32Array(size).fill(-1);
  dist[goalCell] = 0;
  const queue = [goalCell];
  for (let head = 0; head < queue.length; head++) {
    const y = queue[head];
    for (const { dx, dy } of DIRS) {
      const d = dy * w + dx;
      const x = y - d;
      const stand = x - d;
      if (x < 0 || x >= size || stand < 0 || stand >= size) continue;
      if (wall[x] || wall[stand] || dist[x] !== -1) continue;
      dist[x] = dist[y] + 1;
      queue.push(x);
    }
  }
  return dist;
}

// Petak live: ada gol yang bisa dicapai satu kotak sendirian dari petak itu.
function liveMask(wall, w, size, goals) {
  const live = new Uint8Array(size);
  for (const g of goals) {
    const dist = pullDistances(wall, w, size, g);
    for (let i = 0; i < size; i++) if (dist[i] !== -1) live[i] = 1;
  }
  return live;
}

// ---------- wilayah jangkauan pemain ----------
// blocked = dinding ATAU kotak. Mengembalikan { seen, canon } dengan canon
// petak terkecil yang terjangkau (posisi kanonik pemain untuk kunci keadaan).
function reachable(wall, boxSet, w, size, start) {
  const seen = new Uint8Array(size);
  seen[start] = 1;
  let canon = start;
  const stack = [start];
  while (stack.length > 0) {
    const c = stack.pop();
    if (c < canon) canon = c;
    for (const { dx, dy } of DIRS) {
      const n = c + dy * w + dx;
      if (n < 0 || n >= size || seen[n] || wall[n] || boxSet.has(n)) continue;
      seen[n] = 1;
      stack.push(n);
    }
  }
  return { seen, canon };
}

// Jalan BFS pemain dari `from` ke `to` menghindari dinding dan kotak.
// Mengembalikan string huruf kecil, atau null bila tak terjangkau.
function walkPath(wall, boxSet, w, size, from, to) {
  if (from === to) return '';
  const prev = new Int32Array(size).fill(-2);
  prev[from] = -1;
  const queue = [from];
  for (let head = 0; head < queue.length; head++) {
    const c = queue[head];
    for (let di = 0; di < 4; di++) {
      const { dx, dy, ch } = DIRS[di];
      const n = c + dy * w + dx;
      if (n < 0 || n >= size || prev[n] !== -2 || wall[n] || boxSet.has(n)) continue;
      prev[n] = c;
      if (n === to) {
        let path = ch;
        for (let t = c; t !== from; t = prev[t]) {
          const p = prev[t];
          const dd = t - p;
          path = (dd === -w ? 'u' : dd === w ? 'd' : dd === -1 ? 'l' : 'r') + path;
        }
        return path;
      }
      queue.push(n);
    }
  }
  return null;
}

// ---------- heuristik pencocokan bipartit ----------
function allPermutations(n) {
  const out = [];
  const cur = [];
  const used = new Array(n).fill(false);
  const build = () => {
    if (cur.length === n) { out.push(cur.slice()); return; }
    for (let i = 0; i < n; i++) {
      if (used[i]) continue;
      used[i] = true; cur.push(i); build(); cur.pop(); used[i] = false;
    }
  };
  build();
  return out;
}

// ---------- pencarian atas dorongan ----------
// board: { w, size, wall, goals (array terurut), goalSet, boxes, player }
// mode 'astar': prioritas f = g + h. mode 'greedy': prioritas h lalu g.
// Mengembalikan daftar dorongan [{from, to, dir}] atau null.
function searchPushes(board, mode, budget) {
  const { w, size, wall, goals, goalSet } = board;
  const n = board.boxes.length;
  const dists = goals.map((g) => pullDistances(wall, w, size, g));
  const live = new Uint8Array(size);
  for (const d of dists) for (let i = 0; i < size; i++) if (d[i] !== -1) live[i] = 1;
  for (const b of board.boxes) if (!live[b]) return null;

  const perms = n <= 6 ? allPermutations(n) : null;
  const heuristic = (boxes) => {
    if (perms) {
      let best = Infinity;
      for (const perm of perms) {
        let sum = 0;
        for (let i = 0; i < n; i++) {
          const c = dists[perm[i]][boxes[i]];
          if (c === -1) { sum = Infinity; break; }
          sum += c;
          if (sum >= best) break;
        }
        if (sum < best) best = sum;
      }
      return best;
    }
    let sum = 0;
    for (let i = 0; i < n; i++) {
      let m = Infinity;
      for (let gi = 0; gi < n; gi++) {
        const c = dists[gi][boxes[i]];
        if (c !== -1 && c < m) m = c;
      }
      if (m === Infinity) return Infinity;
      sum += m;
    }
    return sum;
  };

  // Blok beku 2x2 yang memuat petak dest: empat petak dinding/kotak dengan
  // sedikitnya satu kotak di luar gol berarti kotak itu tak bisa bergerak lagi.
  const frozenAt = (boxSet, dest) => {
    const bx = dest % w;
    const by = (dest - bx) / w;
    for (const [ox, oy] of [[bx - 1, by - 1], [bx, by - 1], [bx - 1, by], [bx, by]]) {
      if (ox < 0 || oy < 0 || ox + 1 >= w || oy + 1 >= board.h) continue;
      const cells = [oy * w + ox, oy * w + ox + 1, (oy + 1) * w + ox, (oy + 1) * w + ox + 1];
      let allBlocked = true;
      let badBox = false;
      for (const c of cells) {
        if (wall[c]) continue;
        if (boxSet.has(c)) { if (!goalSet.has(c)) badBox = true; continue; }
        allBlocked = false; break;
      }
      if (allBlocked && badBox) return true;
    }
    return false;
  };

  // simpul keadaan
  const sBoxes = [board.boxes.slice()];
  const sPlayer = [board.player];
  const sG = [0];
  const sParent = [-1];
  const sFrom = [-1];
  const sTo = [-1];
  const sDir = [-1];
  const keyOf = (canon, boxes) => String.fromCharCode(canon, ...boxes);
  const startBoxSet = new Set(board.boxes);
  const startReach = reachable(wall, startBoxSet, w, size, board.player);
  const best = new Map([[keyOf(startReach.canon, board.boxes), 0]]);
  const sCanon = [startReach.canon]; // posisi kanonik pemain per simpul, disimpan agar cek basi murah

  // heap biner atas [prioritas, h, counter, stateIdx]
  const heap = [];
  const less = (a, b) => a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hpush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i]; heap[i] = heap[p]; heap[p] = t;
      i = p;
    }
  };
  const hpop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ;) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        const t = heap[i]; heap[i] = heap[c]; heap[c] = t;
        i = c;
      }
    }
    return top;
  };

  const h0 = heuristic(board.boxes);
  if (h0 === Infinity) return null;
  let counter = 0;
  hpush([mode === 'astar' ? h0 : h0, h0, counter++, 0]);
  let expanded = 0;

  while (heap.length > 0 && expanded < budget) {
    const [, , , s] = hpop();
    const boxes = sBoxes[s];
    if (best.get(keyOf(sCanon[s], boxes)) !== sG[s]) continue;
    const boxSet = new Set(boxes);
    const reach = reachable(wall, boxSet, w, size, sPlayer[s]);
    if (boxes.every((b) => goalSet.has(b))) {
      const pushes = [];
      for (let t = s; t > 0; t = sParent[t]) pushes.push({ from: sFrom[t], to: sTo[t], dir: sDir[t] });
      pushes.reverse();
      return pushes;
    }
    expanded++;
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      for (let di = 0; di < 4; di++) {
        const d = DIRS[di].dy * w + DIRS[di].dx;
        const stand = b - d;
        const dest = b + d;
        if (dest < 0 || dest >= size) continue;
        if (!reach.seen[stand]) continue;
        if (wall[dest] || boxSet.has(dest) || !live[dest]) continue;
        const nBoxSet = new Set(boxSet);
        nBoxSet.delete(b); nBoxSet.add(dest);
        if (frozenAt(nBoxSet, dest)) continue;
        const nBoxes = boxes.slice();
        nBoxes[bi] = dest;
        nBoxes.sort((a, c) => a - c);
        const h = heuristic(nBoxes);
        if (h === Infinity) continue;
        const ng = sG[s] + 1;
        const nReach = reachable(wall, nBoxSet, w, size, b);
        const k = keyOf(nReach.canon, nBoxes);
        const seen = best.get(k);
        if (seen !== undefined && seen <= ng) continue;
        best.set(k, ng);
        const t = sBoxes.length;
        sBoxes.push(nBoxes); sPlayer.push(b); sG.push(ng);
        sParent.push(s); sFrom.push(b); sTo.push(dest); sDir.push(di);
        sCanon.push(nReach.canon);
        const prio = mode === 'astar' ? ng + h : h;
        hpush([prio, h, counter++, t]);
      }
    }
  }
  return null;
}

// Putar daftar dorongan menjadi huruf langkah penuh terhadap satu papan.
function pushesToMoves(board, pushes) {
  const boxSet = new Set(board.boxes);
  let player = board.player;
  let out = '';
  for (const p of pushes) {
    const d = DIRS[p.dir].dy * board.w + DIRS[p.dir].dx;
    const stand = p.from - d;
    const walk = walkPath(board.wall, boxSet, board.w, board.size, player, stand);
    if (walk === null) return null;
    out += walk + DIRS[p.dir].ch.toUpperCase();
    boxSet.delete(p.from); boxSet.add(p.to);
    player = p.from;
  }
  return out;
}

function solveSub(board) {
  let pushes = searchPushes(board, 'astar', ASTAR_BUDGET);
  if (pushes === null) pushes = searchPushes(board, 'greedy', GREEDY_BUDGET);
  if (pushes === null) return null;
  return pushesToMoves(board, pushes);
}

// ---------- dekomposisi ruangan ----------
// Potong petak non-live pada baris pemain; komponen lantai yang memuat kotak
// adalah ruangan. Mengembalikan null bila bentuknya tak sesuai harapan.
function decompose(level) {
  const { w, size, wall, goal, boxes, player } = level;
  const goals = [];
  for (let i = 0; i < size; i++) if (goal[i]) goals.push(i);
  const live = liveMask(wall, w, size, goals);
  const prow = Math.floor(player / w);
  const removed = new Uint8Array(size);
  for (let x = 0; x < w; x++) {
    const i = prow * w + x;
    if (!wall[i] && !live[i]) removed[i] = 1;
  }
  const comp = new Int32Array(size).fill(-1);
  let nc = 0;
  for (let i = 0; i < size; i++) {
    if (wall[i] || removed[i] || comp[i] !== -1) continue;
    const stack = [i];
    comp[i] = nc;
    while (stack.length > 0) {
      const c = stack.pop();
      for (const { dx, dy } of DIRS) {
        const n2 = c + dy * w + dx;
        if (n2 < 0 || n2 >= size || wall[n2] || removed[n2] || comp[n2] !== -1) continue;
        comp[n2] = nc;
        stack.push(n2);
      }
    }
    nc++;
  }
  const byComp = new Map();
  for (let i = 0; i < size; i++) {
    if (comp[i] === -1) continue;
    if (!byComp.has(comp[i])) byComp.set(comp[i], { cells: [], boxes: [], goals: [], min: i, hasPlayer: false });
    const o = byComp.get(comp[i]);
    o.cells.push(i);
    if (i < o.min) o.min = i;
    if (i === player) o.hasPlayer = true;
  }
  for (const b of boxes) {
    if (comp[b] === -1) return null; // kotak terpotong dari semua komponen
    byComp.get(comp[b]).boxes.push(b);
  }
  for (const g of goals) {
    if (comp[g] === -1) return null; // gol terpotong
    byComp.get(comp[g]).goals.push(g);
  }
  const rooms = [];
  for (const o of byComp.values()) {
    if (o.boxes.length === 0 && o.goals.length === 0) continue; // sisa lorong
    if (o.boxes.length !== o.goals.length) return null;
    if (o.hasPlayer) return null; // ruangan menyatu dengan posisi pemain
    rooms.push(o);
  }
  if (rooms.length === 0) return null;
  // setiap kotak dan gol harus masuk ruangan
  let nb = 0, ng = 0;
  for (const r of rooms) { nb += r.boxes.length; ng += r.goals.length; }
  if (nb !== boxes.length || ng !== goals.length) return null;
  rooms.sort((a, b) => a.min - b.min);
  for (const r of rooms) { r.boxes.sort((a, b) => a - b); r.goals.sort((a, b) => a - b); }
  return rooms;
}

// Putar ulang langkah di papan nyata untuk memperbarui kotak dan pemain.
function replay(level, boxesSet, player, moves) {
  const { w, wall } = level;
  let p = player;
  const bs = new Set(boxesSet);
  for (const ch of moves) {
    const low = ch.toLowerCase();
    const dir = DIRS.find((d) => d.ch === low);
    const d = dir.dy * w + dir.dx;
    const next = p + d;
    if (wall[next]) return null;
    if (bs.has(next)) {
      const beyond = next + d;
      if (wall[beyond] || bs.has(beyond)) return null;
      bs.delete(next); bs.add(beyond);
    }
    p = next;
  }
  return { boxes: bs, player: p };
}

function solveLevel(text) {
  const level = parseLevel(text);
  const goalsAll = [];
  for (let i = 0; i < level.size; i++) if (level.goal[i]) goalsAll.push(i);
  const rooms = decompose(level);
  if (rooms !== null) {
    let actualBoxes = new Set(level.boxes);
    let actualPlayer = level.player;
    let answer = '';
    let ok = true;
    for (const room of rooms) {
      const subWall = level.wall.slice();
      const roomBoxSet = new Set(room.boxes);
      for (const b of actualBoxes) if (!roomBoxSet.has(b)) subWall[b] = 1;
      const board = {
        w: level.w, h: level.h, size: level.size, wall: subWall,
        goals: room.goals, goalSet: new Set(room.goals),
        boxes: room.boxes.slice(), player: actualPlayer,
      };
      const moves = solveSub(board);
      if (moves === null) { ok = false; break; }
      const after = replay(level, actualBoxes, actualPlayer, moves);
      if (after === null) { ok = false; break; }
      actualBoxes = after.boxes;
      actualPlayer = after.player;
      answer += moves;
    }
    if (ok) return answer;
  }
  // fallback: seluruh level satu submasalah
  const board = {
    w: level.w, h: level.h, size: level.size, wall: level.wall,
    goals: goalsAll, goalSet: new Set(goalsAll),
    boxes: level.boxes.slice(), player: level.player,
  };
  const moves = solveSub(board);
  return moves ?? '';
}

for (const id of readdirSync(IN_DIR).sort()) {
  const moves = solveLevel(readFileSync(join(IN_DIR, id, 'level.txt'), 'utf8'));
  mkdirSync(join(OUT_DIR, id), { recursive: true });
  writeFileSync(join(OUT_DIR, id, 'solution.txt'), `${moves}\n`);
}
