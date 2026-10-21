// Headless runner for tests/test_web_sim_parity.py.
//   node web/parity_cli.mjs '<json>'
// json: {"legs": [["forward", 3.0], ["left", 4.0]], "sample_every": 50,
//        "phrases": ["go straight", ...], "opts": {...Flight options}}
// Prints {"t": [...], "p": [[x,y,z], ...], "classify": [...]} as JSON.
import { PARAMS } from "./params.js";
import { Flight, classify } from "./sim_core.js";

const spec = JSON.parse(process.argv[2]);
const f = new Flight(PARAMS, spec.opts || {});
const out = { t: [], p: [], classify: (spec.phrases || []).map((s) => classify(PARAMS, s)) };
const every = spec.sample_every || 50;
for (const [cmd, secs] of spec.legs || []) {
  f.dispatch(cmd);
  const n = Math.round(secs / PARAMS.DT);
  for (let i = 0; i < n; i++) {
    f.step();
    if (f.k % every === 0) { out.t.push(f.t); out.p.push(f.p); }
  }
}
process.stdout.write(JSON.stringify(out));
