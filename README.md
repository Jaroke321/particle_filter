# particle_filter
Implementation of a particle filter localization algorithm

</br>
Particle_filter.cpp:</br>
  - Original particle filter implementation. Achieves good accuracy but could be better. Needs to account for 0 yaw_rate in prediction function.</br>
Particle_filter1.cpp:</br>
  - updated particle_filter that accounts for division by zero in prediction function for particles. This improves accuracy by a few percentage points for x and y direction.
