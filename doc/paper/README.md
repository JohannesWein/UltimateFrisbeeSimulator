# Flying-disc flight paper

## Bibliographic record

Koyanagi, R., Seo, K., Ohta, K., & Ohgi, Y. (2012). *A computer simulation
of the flying disc based on the wind tunnel test data*. Procedia Engineering,
34, 80–85. https://doi.org/10.1016/j.proeng.2012.04.015.

The paper is stored locally as
[`A_computer_simulation_of_the_flying_disc_based_on_.pdf`](./A_computer_simulation_of_the_flying_disc_based_on_.pdf).

## What the paper measures

The authors report measuring the three-axis forces and moments acting on a
disc in a wind tunnel. The measured data comprise all three force components
and all three moment components, reported as aerodynamic coefficient curves
over the test conditions. The reported wind-tunnel sweep rotates the disc
about a principal inertia axis from 0 to 14 revolutions per second, varies
angle of attack from +90 to -10 degrees, and varies air speed from 0 to
20 m/s. The paper presents these results in plots; no tabulated numerical
dataset has been transcribed into this repository, so no coefficient values
from those plots are currently used by the simulator.

The paper distinguishes the wind-tunnel disc from the disc used for the
motion-capture flight comparison: it says the latter was 60 g heavier than a
"normal" disc and had greater inertia. The authors discuss this physical
difference as a possible source of simulation error, particularly in the
force-axis comparisons. Therefore, the wind-tunnel measurements and flight
validation do not establish one interchangeable set of disc properties. The
text reviewed does not identify the wind-tunnel disc by manufacturer, model,
or absolute mass, nor does it identify the flight-comparison disc as a
175 g model.

## Use in this simulator

The paper supports a six-component force-and-moment model, spin-dependent
testing, and flight-trajectory validation. It does not identify either disc
as a Discraft Ultra-Star in the text reviewed for this implementation. Its
measurements must not be labelled UltraStar-specific unless the disc model is
confirmed from the paper or an independent source.

WFDF's [approved-disc list](https://wfdf.sport/organisation/discs-targets/)
includes both the Discraft UltraStar and the **eurodisc 175g Organic 4.1**
at Championship Level. This establishes that the named Eurodisc model is an
approved 175 g Ultimate disc; it does not establish that its aerodynamic
coefficients match the UltraStar or either disc tested by Koyanagi et al.
Weizman et al. (2020), DOI
[10.3390/proceedings2020049066](https://doi.org/10.3390/proceedings2020049066),
report a 210 g disc, so that flight-dynamics measurement is comparative rather
than direct evidence for a 175 g disc.

Any future coefficient values digitized or transcribed from Koyanagi et al.
must record the source figure, test conditions, and disc-equivalence
assumption alongside the values. Until coefficients are measured for or
calibrated against a named model, the UltraStar inertia and aerodynamic
parameters remain explicit assumptions.
