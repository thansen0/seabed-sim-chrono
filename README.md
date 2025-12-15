# Deep Sea Polymetallic Nodule Sim in Chrono

Many years ago I worked on a polymetallic nodule startup. Most of my work at the time was related to hardware (or frankly legal) things, but I've always wanted to build a simulation of the seabed using Project Chrono and leftover polymetallic nodules I kept after the company shut down. 

Now is that time. 

This is very much an early stage project, and so much of the code is just seeing what runs well on my hardware or getting used to the libraries again. As the project progresses, I'll update this readme.

## Immediate Goals

 - ~~isolate into separate src/thing folders~~
 - try deformable terrain
   - decide on terrain
   - possibly add veh to cmakelist.txt?
 - space out objects
 - load in different shaped rigid object
                                                        
Possible future tasks?
 - see if there's a better way to add things other than dropping them (polymetallic nodules aren't dropped onto a surface)

## Install

This project (so far at least) uses project chrono version 9, and uses the components VSG and Multicore. You will have to set 

```
set(Chrono_DIR "$ENV{HOME}/Code/seabed_sim/chrono/build/cmake")
```

In your cmake to match wherever you have chrono installed/set up. Further when you build chrono you will likely have to configure VSG, and when doing that I highly recommend using the VSG setup script in Chrono, and specifically using the script in the chrono version you use.

One point of note is the script uses the most recent version of vsgImGui, which broke for me. I had to specify v0.6.0 by modifying the script. 

