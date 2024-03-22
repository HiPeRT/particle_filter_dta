#include "particle_filter.hpp" 

bool ParticleFilter::wayToSort(Particle_t i, Particle_t j) { 
  return i.weight > j.weight;
}