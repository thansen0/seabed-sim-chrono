# Deep sea Sim in Chrono

# Immediate Goals for Project

 - isolate into separate src/thing folders (like demo)
 - try deformable terrain
   - decide on terrain
   - possibly add veh to cmakelist.txt?
 - space out objects
 - load in different shaped rigid object
                                                        
Possible future tasks?
 - see if there's a better way to add things other than dropping them (rocks aren't dropped onto a surface lol)

## Install

This project (so far at least) uses project chrono version 9, and uses the components VSG and Multicore. You will have to set 

```
set(Chrono_DIR "/home/thomas/Code/seabed_sim/chrono/build/cmake")
```

In your cmake to match wherever you have chrono installed/set up. Further when you build chrono you will likely have to configure VSG, and when doing that I highly recommend using the VSG setup script in Chrono, and specifically using the script in the chrono version you use.

One point of note is the script uses the most recent version of vsgImGui, which broke for me. I had to specify v0.6.0 by modifying the script. 

