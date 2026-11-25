# Teaching an AI to Land a Rocket in Gusty Wind

A 2-D rocket-landing simulator that tests whether a reinforcement-learning pilot trained with **domain randomisation** lands more reliably in unseen wind than a pilot trained in calm air or a classical PID controller.

**[Live demo](https://nipunmeegoda.github.io/rl-rocket-simulator/)** · [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/nipunmeegoda/rl-rocket-simulator/blob/main/notebook/rl-rocket-simulator.ipynb)

![Screenshot of the web demo](figures/demo-screenshot.png)

## The problem

Reusable rockets such as SpaceX's Falcon 9 now land vertically under automated guidance, and reinforcement learning (RL) is being explored as an alternative to hand-tuned controllers. A known weakness of learned controllers is that a policy trained in one simulated world can fail when conditions change. Robustness to the unexpected is one of the main barriers to trusting AI controllers in safety-critical systems.

This project asks one question: **does training with domain randomisation (randomised mass, gravity and wind in every episode) make a Deep Q-Network (DQN) agent land more reliably under unseen wind conditions than (a) the same agent trained in calm conditions and (b) a classical PID controller?** Each controller is scored by its landing success rate over 1,000 Monte Carlo test landings, reported with 95% confidence intervals.

## Quick start

**Notebook:** click the *Open in Colab* badge above, then *Runtime → Run all*. By default the notebook loads pre-trained weights, so a full run takes about a minute on a laptop (Apple M1 Pro). Set `RETRAIN = True` to train the 20 main agents in the notebook (about 40 minutes), or run `train.py` to train agents in parallel.

**Web demo:** open the [live demo](https://nipunmeegoda.github.io/rl-rocket-simulator/), or download this repository and double-click `docs/index.html`.

**Run locally (optional):**

```bash
pip install -r requirements.txt
jupyter notebook notebook/rl-rocket-simulator.ipynb
```

## Using the demo

Choose a controller (PID, calm-trained DQN or domain-randomised DQN), choose the conditions (or set wind, gusts, mass and gravity yourself) and a random seed, then press **Start** to watch a landing. Run a batch of episodes to estimate the success rate with its confidence interval. Full guide: [USER_GUIDE.md](USER_GUIDE.md).

## Model and mathematics

The rocket is a 2-D rigid body with state $(x, y, v_x, v_y, \theta, \omega, m)$. Translation follows Newton's second law under thrust, gravity and aerodynamic drag relative to a stochastic wind; rotation follows $I\,\dot\omega = \tau$ from the side thrusters; and fuel burns as

$$\frac{dm}{dt} = -\frac{T}{I_{sp}\, g_0}.$$

The equations are integrated with the classical fourth-order Runge–Kutta method (RK4). The learning agent is a DQN with a discrete action set (main engine, side thrusters and their combinations) and a shaped reward. Full derivations, assumptions and parameters: [THEORY.md](THEORY.md).

## Verification

The simulator is checked against closed-form results before any controller is trained:

- free fall matches $y(t) = y_0 + v_0 t - \tfrac{1}{2} g t^2$;
- a constant-thrust burn matches the Tsiolkovsky rocket equation $\Delta v = I_{sp} g_0 \ln(m_0/m_1)$;
- torque-free motion keeps $\omega$ constant, and mechanical energy is conserved when thrust and drag are off;
- the RK4 error falls as $O(\Delta t^4)$ on a log–log plot.

Measured results: the free-fall error is at most $1.1\times10^{-13}$ m, the Tsiolkovsky $\Delta v$ agrees to a relative error of $6\times10^{-14}$, energy is conserved to $1.6\times10^{-15}$ (relative), the fitted RK4 order is 4.00 (forward Euler 0.94) and the Monte Carlo error falls with slope −0.49 (theory −0.5). All checks are assertions in the notebook; the full list is in [THEORY.md](THEORY.md#5-verification).

![Verification plot](figures/verification.png)

## Concepts applied

This project applies reinforcement learning (Deep Q-Networks with experience replay and a target network) and domain randomisation for sim-to-real-style robustness; numerical integration of ordinary differential equations with the fourth-order Runge–Kutta method, including a measured convergence-order check; input modelling and random variate generation for stochastic wind gusts and randomised physical parameters; and Monte Carlo estimation of success probabilities with confidence intervals based on the law of large numbers. The simulator is verified against analytic free-fall, rocket-equation and conservation results.

## Results

**Randomisation within the training range did not make the DQN measurably more reliable in unseen wind, and the PID controller landed most often.** The table gives landing success over 1,000 test episodes. DQN figures are means over ten training seeds with 95% intervals over seeds; PID intervals are Wilson intervals over episodes.

| Test conditions | PID | DQN, calm | DQN, randomised |
|---|---|---|---|
| Calm | 100.0% [99.6, 100.0] | 99.3% [98.7, 99.9] | 99.4% [98.9, 99.9] |
| Training conditions (wind up to 8 m/s, mass and gravity varied) | 100.0% [99.6, 100.0] | 70.5% [59.1, 82.0] | 98.3% [97.1, 99.6] |
| **Unseen wind (10–15 m/s, gusts 3–5 m/s)** | **92.1% [90.3, 93.6]** | **76.1% [66.9, 85.2]** | **74.6% [63.2, 86.1]** |

In unseen wind the randomised DQN differs from the calm DQN by −1.4 points [−15.1, +12.2] and from the PID by −17.5 points [−28.9, −6.0].

![Landing success by controller and scenario](figures/success_by_scenario.png)

- **Wind strength:** the two DQNs behave alike at every wind level, and the PID is ahead from 10 to 17.5 m/s.
- **What randomisation did help:** robustness to mass. With mass alone varied, the calm DQN lands 70.9% and the randomised DQN 98.6%.
- **Randomisation range:** agents trained on wind up to 12 and 16 m/s land 92.1% [88.6, 95.6] and 94.7% [92.9, 96.6] in the same test. Their training wind overlaps the test wind, so for them it is no longer unseen.

![Landing success against wind strength](figures/success_vs_wind.png)

![Success in unseen wind against randomisation range](figures/success_vs_range.png)

## Limitations and future work

- The model is 2-D and rigid: there is no out-of-plane motion, fuel slosh, structural flexing or engine gimbal dynamics, and all physical parameters are synthetic rather than taken from a real vehicle.
- Training varies a lot between seeds (40.9% to 93.2% in unseen wind for the randomised DQN), and each agent is its best checkpoint on its own training scenario. Seeds 5–9 were added, and the training gravity range was narrowed, after earlier results had been seen; details are in [THEORY.md](THEORY.md).
- Wind exerts no torque on the rocket in this model, which makes gusts easier than in reality.
- The PID has integral memory and the DQN does not, which may explain part of the gap; this was not tested.
- Actions are discrete (engines on or off), whereas real landing engines throttle continuously.
- Results are from simulation only; a policy that is robust here is not proven robust on hardware.
- Planned extension: noisy sensors with a Kalman filter that estimates the state before it is passed to the agent.

## Key references

1. V. Mnih *et al.*, "Human-level control through deep reinforcement learning," *Nature*, vol. 518, no. 7540, pp. 529–533, 2015, doi: 10.1038/nature14236.
2. J. Tobin *et al.*, "Domain randomization for transferring deep neural networks from simulation to the real world," in *Proc. IEEE/RSJ IROS*, 2017, pp. 23–30, doi: 10.1109/IROS.2017.8202133.
3. R. S. Sutton and A. G. Barto, *Reinforcement Learning: An Introduction*, 2nd ed. MIT Press, 2018.

Full list: [REFERENCES.md](REFERENCES.md).

## AI assistance

Developed with the help of AI coding and writing assistants. All code was reviewed, run and verified by the author.

## Licence and credits

Code: [MIT](LICENSE). Data: no external datasets are used; all physical parameters are synthetic and stated in [THEORY.md](THEORY.md).
