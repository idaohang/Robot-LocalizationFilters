#ifndef PARTICLE_FILTER_HELPER_H
#define PARTICLE_FILTER_HELPER_H

#include <eigen3/Eigen/Core>
#include <algorithm>

template<typename Particle>
class CompareParticle2D {
public:
	bool operator() (const Particle& lhs, const Particle& rhs)
	{
		return (lhs(0) < rhs(0)) || (lhs(0) == rhs(0) && lhs(1) == rhs(1));
	}
};

template<typename Particle,
	typename MeanWindow>
class ClimbFunctor2D {
public:
	typedef std::vector<Particle> ParticleSet;

	ParticleSet& ps_;

	ClimbFunctor2D(ParticleSet& ps)
		:ps_(ps)
	{
	}
	
	bool climb(MeanWindow& mw)
	{
		Particle tl(mw.center), br(mw.center);
		tl(0) -= mw.radius;
		tl(1) -= mw.radius;
		br(0) += mw.radius;
		br(1) += mw.radius;
		auto lower = std::lower_bound(ps_.begin(), ps_.end(), tl, cmpobj_);
		auto upper = std::upper_bound(lower, ps_.end(), br, cmpobj_);
		mw.nparticles = 0;
		Particle newmean;
		for (auto iter = lower; iter < upper; iter++) {
			Particle& p(*iter);
			if (p(1) >= tl(1) && p(1) <= br(1)) {
				++mw.nparticles;
				newmean += p;
			}
		}
		bool ret = true;
		if (abs(mw.center(0) - newmean(0)) + abs(mw.center(1) - newmean(1)) < 5.0)
			ret = false;
		mw.center = newmean;
		if (mw.nparticles > 0.25 * ps_.size())
			mw.radius *= 0.75;
		return ret;
	}

private:
	CompareParticle2D<Particle> cmpobj_;
};

#endif
