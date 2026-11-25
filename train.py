"""Train the DQN agents in parallel and save their weights.

The simulator and training code are read from the notebook, so there is one copy of them.
The notebook itself loads the saved weights and does not need this script.

    pip install -r requirements.txt
    python train.py                          # all 20 agents -> models/dqn_weights_new.pt
    python train.py --compare models/dqn_weights.pt
    python train.py --agents "calm DQN" --seeds 0 1 --workers 2
    python train.py --agents "range 0.5" "range 1.5" "range 2" --out models/dqn_range_weights.pt
    python train.py --watch --agents "calm DQN" --seeds 0      # one agent, in a live window
"""
import argparse
import contextlib
import io
import json
import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from multiprocessing import get_context
from pathlib import Path

ROOT = Path(__file__).resolve().parent
NOTEBOOK = ROOT / "notebook" / "rl-rocket-simulator.ipynb"
STOP = "RETRAIN = False"          # first line of code that is about loading weights, not training
_ns = {}


def notebook_code():
    """Run the notebook's code cells up to the weight-loading cell; returns their names."""
    if not _ns:
        cells = json.loads(NOTEBOOK.read_text(encoding="utf-8"))["cells"]
        for cell in cells:
            source = "".join(cell["source"])
            if cell["cell_type"] != "code":
                continue
            if STOP in source:
                break
            with contextlib.redirect_stdout(io.StringIO()):
                exec(compile(source, f"notebook cell {cell.get('id')}", "exec"), _ns)
        else:
            raise RuntimeError(f"{STOP!r} not found in the notebook")
    return _ns


class Progress:
    """Prints one line per validation."""
    def __init__(self, name, seed, total):
        self.label, self.total, self.t0, self.best = f"{name} {seed}", total, time.time(), 0.0

    def __call__(self, step, eps, rate, net):
        self.best = max(self.best, rate)
        done, el = step / self.total, time.time() - self.t0
        bar = "#" * round(20 * done)
        print(f"{self.label:13s} [{bar:<20s}] {100 * done:3.0f}%  exploring {100 * eps:3.0f}%  landing {100 * rate:5.1f}%"
              f"  best {100 * self.best:5.1f}%  {el:4.0f} s, about {el / done - el:3.0f} s left", flush=True)


class Watch(Progress):
    """Also draws the current policy's flights and the learning curve in a window."""
    N_FLIGHTS = 25

    def __init__(self, name, seed, total, scenario, ns):
        super().__init__(name, seed, total)
        import matplotlib.pyplot as plt
        self.plt, self.scenario, self.ns, self.steps, self.rates = plt, scenario, ns, [], []
        plt.ion()
        self.fig, self.ax = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")

    def flights(self, net):
        """Paths of 25 fixed episodes flown greedily; separate seed, so training is unaffected."""
        ns, np = self.ns, self.ns["np"]
        env, ctrl = ns["RocketEnv"](self.N_FLIGHTS, self.scenario, 7), ns["DQNController"](net)
        path = [env.s[:, :2].copy()]
        while (env.outcome == ns["FLYING"]).any():
            s, _ = env.step(ctrl.act(env.s))
            path.append(s[:, :2])
        return np.array(path), env.outcome == ns["SUCCESS"]

    def __call__(self, step, eps, rate, net):
        super().__call__(step, eps, rate, net)
        self.steps.append(step / self.total * 100)
        self.rates.append(100 * rate)
        if not self.plt.fignum_exists(self.fig.number):     # window closed: keep training
            return
        path, ok = self.flights(net)
        a, b = self.ax
        a.clear(), b.clear()
        for i in range(self.N_FLIGHTS):
            a.plot(path[:, i, 0], path[:, i, 1], color="#2a78d6" if ok[i] else "#eb6834", lw=1.2,
                   ls="-" if ok[i] else "--")
            a.plot(*path[-1, i], "o" if ok[i] else "x", color="#2a78d6" if ok[i] else "#eb6834", ms=6)
        a.plot([-10, 10], [0, 0], color="#0b0b0b", lw=5, solid_capstyle="butt")
        a.set(xlim=(-60, 60), ylim=(-5, 130), xlabel="x [m]", ylabel="height [m]",
              title=f"Current policy: {ok.sum()} of {self.N_FLIGHTS} land (solid: landed, dashed: failed)")
        b.plot(self.steps, self.rates, color="#2a78d6", lw=2)
        b.set(xlim=(0, 100), ylim=(0, 102), xlabel="training progress (%)", ylabel="landing success (%)",
              title=f"{self.label}: {100 * rate:.1f}% now, best {100 * self.best:.1f}%, exploring {100 * eps:.0f}%")
        for x in self.ax:
            x.grid(alpha=0.25)
        self.plt.pause(0.01)


def train_one(name, seed, quiet=False, watch=False):
    ns = notebook_code()
    if name.startswith("range "):
        scenario = ns["randomised"](float(name.split()[1]))
    else:
        scenario = {"calm DQN": ns["CALM"], "robust DQN": ns["DR_TRAIN"]}[name]
    t0, total = time.time(), ns["CFG"].env_steps
    progress = Watch(name, seed, total, scenario, ns) if watch else None if quiet else Progress(name, seed, total)
    state, history, val = ns["train_dqn"](scenario, seed, callback=progress)
    # plain arrays and floats are safe to send between processes
    return name, seed, {k: v.numpy() for k, v in state.items()}, history, val, time.time() - t0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--agents", nargs="+", default=["calm DQN", "robust DQN"], choices=["calm DQN", "robust DQN", "range 0.5", "range 1.5", "range 2"])
    ap.add_argument("--seeds", nargs="+", type=int, default=list(range(10)))
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", type=Path, default=ROOT / "models" / "dqn_weights_new.pt")
    ap.add_argument("--compare", type=Path, help="existing weights file to compare the result with")
    ap.add_argument("--quiet", action="store_true", help="no progress lines")
    ap.add_argument("--watch", action="store_true", help="show one agent training in a window")
    args = ap.parse_args()
    if args.watch and len(args.agents) * len(args.seeds) != 1:
        ap.error("--watch needs exactly one agent and one seed, e.g. --agents \"calm DQN\" --seeds 0")

    import torch
    old = torch.load(args.compare, weights_only=True) if args.compare else None
    jobs = [(name, seed) for name in args.agents for seed in args.seeds]
    found = {}
    t0 = time.time()

    def collect(result):
        name, seed, state, history, val, secs = result
        found[f"{name} {seed}"] = dict(state={k: torch.from_numpy(v) for k, v in state.items()},
                                       history=torch.tensor(history), val=val)
        print(f"DONE {name}, seed {seed}: validation {val:.3f}, {secs:.0f} s", flush=True)

    if args.watch:
        collect(train_one(*jobs[0], watch=True))
    else:
        os.environ["MPLBACKEND"] = "Agg"        # workers draw nothing
        with ProcessPoolExecutor(args.workers, mp_context=get_context("spawn")) as pool:
            for fut in as_completed([pool.submit(train_one, *job, args.quiet) for job in jobs]):
                collect(fut.result())
    saved = {f"{name} {seed}": found[f"{name} {seed}"] for name, seed in jobs}     # fixed order
    args.out.parent.mkdir(parents=True, exist_ok=True)
    torch.save(saved, args.out)
    print(f"saved {len(saved)} agents to {args.out} in {time.time() - t0:.0f} s")

    if old is not None:
        same = [k for k in saved if k in old
                and all(torch.equal(old[k]["state"][n], w) for n, w in saved[k]["state"].items())]
        print(f"identical to {args.compare}: {len(same)} of {len(saved)}")
        for k in saved:
            if k not in same:
                note = f"validation {old[k]['val']:.3f} -> {saved[k]['val']:.3f}" if k in old else "not in that file"
                print(f"  differs: {k} ({note})")
    if args.watch:
        import matplotlib.pyplot as plt
        if plt.get_fignums():
            print("close the window to finish")
            plt.ioff()
            plt.show()


if __name__ == "__main__":
    main()
