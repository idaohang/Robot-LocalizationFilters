#ifndef MEAN_WINDOW_H
#define MEAN_WINDOW_H

namespace filter {
template<typename Particle>
struct MeanWindow {
	Particle center;
	size_t nparticles;
	double radius;
};
}

#endif
