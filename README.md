# 2020
## Vision
The 2020 stack is being redesigned to berun efficeiently on low resource hardware and is specifically targetted to run on a Raspberry Pi 4. The new roboo dimensions from NASA's RMC made using a laptop hard. As part of this effort, the 2020 stack will be broken into multiple sub-systems that are designed to perform one task (Localization, path planning, obstacel detection etc) to allow for
- Easier debugging
- Faster and parallel interation
- Running across multiple cores

### Data Exchange
Running across multiple cores and also being able to iterate in parallel on different subsystems lead us to choosing to keep each subsystem a different procvess (instead of threads). But due to the nature of a robotics control stack, being able to efficiently exchange quickly changing data between modules is crucial. this stack exchnages data between sub-systems via a [custom MMAP based file data exchange](autonomy/include/aurora/data_exchange.h)
