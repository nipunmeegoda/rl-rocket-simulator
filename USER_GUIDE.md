# User guide: Teaching an AI to Land a Rocket in Gusty Wind (web demo)

**Open the demo:** [https://nipunmeegoda.github.io/rl-rocket-simulator/](https://nipunmeegoda.github.io/rl-rocket-simulator/). It runs entirely in your browser (current Chrome, Edge or Firefox). Nothing is installed or uploaded. You can also download the repository and double-click `docs/index.html`.

![Labelled screenshot of the demo](figures/demo-screenshot-labelled.png)

## Controls

| Control | What it does | Range | Default |
|---|---|---|---|
| Controller | Which pilot flies the rocket: PID, DQN trained in calm air, or DQN trained with domain randomisation | — | PID |
| Conditions | Calm, training conditions, unseen wind, or custom (sliders) | — | Unseen wind |
| Mean wind | Average horizontal wind; positive blows to the right | −25 to 25 m/s | drawn from the conditions |
| Gust strength | Standard deviation of the wind around its mean | 0 to 8 m/s | drawn from the conditions |
| Starting mass | Mass of the rocket including 100 kg of fuel | 900 to 1100 kg | 1000 kg |
| Gravity | Local gravity | 9.3 to 10.3 m/s² | 9.81 m/s² |
| Random seed | Fixes the start position and wind so a run can be repeated exactly | integer ≥ 1 | 42 |
| Playback speed | Speed of the animation only | 0.5× to 5× | 1× |
| Start / Pause | Runs or pauses the flight | — | — |
| Reset | Returns to the start of the flight | — | — |
| Landings each, Run batch | Flies many landings with all three controllers | 100, 500 or 1000 | 1000 |

Moving a condition slider switches **Conditions** to *Custom*.

## Reading the outputs

- **Rocket view:** the rocket (drawn larger than scale), its engine flames, the landing pad, an arrow for the current wind, a close-up and the path flown.
- **Numbers below the view:** live values. At touchdown, the four values with landing limits turn green or red: vertical speed (2 m/s), sideways speed (1 m/s), tilt (10°) and distance from the pad (10 m).
- **Outcome:** *Landed* if all four limits are met, otherwise *Crashed* with the reason, or *Failed* (out of fuel, out of area or time limit).
- **Success rate table:** the fraction of landings that succeeded, with a 95% confidence interval (the black line on each bar). The interval narrows roughly as $1/\sqrt{N}$.

## Try this

1. Keep the defaults and press **Start**. The PID lands after about 15 s. Choose *DQN, trained in calm air* and press **Start** again: in the same wind it misses the pad.
2. Press **Run batch**. With seed 42 the success rates should be 91.5% (PID), 79.3% (calm DQN) and 58.5% (randomised DQN).
3. Change **Conditions** to *Training conditions* and run the batch again. The DQN trained in calm air now lands only about half the time, while the other two land almost always.

## Check against the notebook

With the default settings, a batch of 1,000 gives a PID success rate that should match the notebook's **92.1% to within ±3.6 percentage points** (notebook section 5.1). The table also checks the two DQNs. The random numbers differ from the notebook's, so the tolerance is three standard errors of the difference between two samples.

Each controller also flies one fixed, wind-free flight when the page opens. Its touchdown values must match notebook section 5.5 to within 0.001.

## Limitations

- The demo runs the same physics as the notebook, but in JavaScript, so tiny floating-point differences can change individual episodes; batch success rates agree within the stated tolerance.
- The DQN policies are the networks trained in the notebook, exported to the page; they are not retrained in the browser.
- The two DQNs are training seed 0 of ten. Seeds differ widely: in unseen wind the ten-seed averages are 76.1% (calm) and 74.6% (randomised), so the single networks shown here should not be read as the overall comparison.
- Wind stronger than 15 m/s is outside the conditions tested in the notebook, and the DQNs are unreliable far outside their training range (wind up to 8 m/s).
