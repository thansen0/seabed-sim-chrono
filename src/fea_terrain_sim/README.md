# Notes on FEA Sim

It turns out the parallelism for FEA is so bad it's basically unusable. While this simulation will render everything and turn on, it's painfully slow on even the fastest of computers. Additionally, although it sounds promising, `ChSystemMulticoreSMC` doesn't work for FEA. If I add 100 "spheres" (basically easy to simulate versions of polymetallic nodules) I can't run it with multicore, defeating the purpose.

Additionally, collision is *not* working in this sim. It takes forever to get to that point, but I think I need to explicitly add a solver. The problem is the solver support is so poor the two I tried crashed/didn't work. The default one (whichever that is) seems to also be having issues.

For these reason, I'm giving up on FEA. I will keep rigid, and I plan to try DEM/Granular.
