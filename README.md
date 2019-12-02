# 2020
## Refactor

This is the refactor branch. The main goal of this exercise is to decouple all subsystems from the current monolithic [backend](autonomy/backend/main.cpp).

### Data Exchange
Due to the nature of a robotics control stack, being able to efficiently exchange quickly changing data between modules is crucial. We plan to do this using the new MMAP-based shared file data exchange system. The code for this can be found in the [data_exchange.h](autonomy/include/aurora/data_exchange.h) header.