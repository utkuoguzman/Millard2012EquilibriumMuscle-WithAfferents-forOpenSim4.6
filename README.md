* OpenSim 4.6 had some changes regarding how the plugins work.
* It has poor documentation about the possible coding options of running stuff variously.
* So with lots of trial and error (with the help of Gemini), I made the original Millard12EdMuscleWithAfferents.cpp codes work in OpenSim 4.6.
* This plugin works with the explicit solvers. However the equations are stiff, hence ideally the new implicit solver would be the best option, because it introduces some oscillations. Unfortunately the new implicit solver of OpenSim 4.6 requires *really* small time steps at the beginning (1e-8) to work properly. Meaning if you have a goal of real time factor (mine is 1:70 between the simulated seconds and the wall time), then implicit method doesn't satisfy this (more like 1:3900).
* My interest is neck and eye muscles, so the repo also has an .osim which uses the HYOID_Scaled model and the Upastras model.

* Use anything to your liking. Share if you do cool stuffs with it.
