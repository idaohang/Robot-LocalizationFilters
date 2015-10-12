#include "ParticleFilter.h"
#include <cmath>

// 1D PF
typedef ParticleFilter<1000, 1, 1, 1> PF;
typedef typename PF::Motion Motion;
typedef typename PF::Particle Particle;
typedef typename PF::Ob Ob;

class MM {
public:
	// Given robot's motion u and particle in the previous previous frame
	// Return the anticipated location of this particle in current frame
	Particle operator()(const Motion& u, const Particle& x)
	{
		return x - u;
	}
};

class ObM {
	static constexpr double theta = 5.0;
	static constexpr double theta2 = theta*theta;
public:
	// Given robot's observation z, and (assumed) ground truth x
	// Return p(z|x)
	double operator()(const Ob& z, const Particle& x)
	{
		double err = z(0,0) - x(0,0);
		double e = -(err*err)/(2*theta2);
		double frac = 1/(theta*std::sqrt(2*M_PI));
		return frac * std::exp(e);
	}
};

int main(int argc, char* argv[])
{
	PF pf;
	Motion m;
	m << 1.0;
	Ob ob;
	ob << 50.0;
	pf.filter(m, ob, MM(), ObM());
	return 0;
}
