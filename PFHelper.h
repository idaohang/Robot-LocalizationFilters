#ifndef PARTICLE_FILTER_HELPER_H
#define PARTICLE_FILTER_HELPER_H

#include <eigen3/Eigen/Core>
#include <algorithm>
#include "MeanWindow.h"

namespace filter {

template<typename Particle>
class CompareParticle1D {
public:
	bool operator() (const Particle& lhs, const Particle& rhs)
	{
		return lhs(0) < rhs(0);
	}
};

template<typename Particle>
class ClimbFunctor1D {
public:
	typedef std::vector<Particle> ParticleSet;
	typedef MeanWindow<Particle> MW;

	ParticleSet& ps_;

	ClimbFunctor1D(ParticleSet& ps)
		:ps_(ps)
	{
	}
	
	bool operator() (MW& mw)
	{
		Particle tl(mw.center), br(mw.center);
		tl(0) -= mw.radius;
		br(0) += mw.radius;
		auto lower = std::lower_bound(ps_.begin(), ps_.end(), tl, cmpobj_);
		auto upper = std::upper_bound(lower, ps_.end(), br, cmpobj_);
		mw.nparticles = 0;
		Particle newmean(Particle::Zero());
		printf("\tLV: %f, HV: %f, Left: %f (at %lu), Right: %f (at %lu)\n",
				tl(0), br(0),
				(*lower)(0), lower-ps_.begin(),
				(*upper)(0), upper-ps_.begin()
			);
		for (auto iter = lower; iter < upper; iter++) {
			Particle& p(*iter);
			++mw.nparticles;
			newmean += p;
		}
		bool ret = true;

		// Update center and radius accordingly
		if (mw.nparticles) {
			newmean /= mw.nparticles;
			if (abs(mw.center(0) - newmean(0)) < 1.0)
				ret = false;
			mw.center = newmean;
			if (mw.nparticles > 0.25 * ps_.size())
				mw.radius *= 0.75;
		} else {
			mw.radius *= 1.25;
		}

		printf("\t\tNew mean: %f\n", newmean(0));
		return ret;
	}

private:
	CompareParticle1D<Particle> cmpobj_;
};

template<typename Particle>
class CompareParticle2D {
public:
	bool operator() (const Particle& lhs, const Particle& rhs)
	{
		return (lhs(0) < rhs(0)) || (lhs(0) == rhs(0) && lhs(1) < rhs(1));
	}
};

template<typename Particle>
class ClimbFunctor2D {
public:
	typedef std::vector<Particle> ParticleSet;
	typedef MeanWindow<Particle> MW;

	ParticleSet& ps_;

	ClimbFunctor2D(ParticleSet& ps)
		:ps_(ps)
	{
	}
	
	bool operator() (MW& mw)
	{
		Particle tl(mw.center), br(mw.center);
		tl(0) -= mw.radius;
		tl(1) -= mw.radius;
		br(0) += mw.radius;
		br(1) += mw.radius;
		auto lower = std::lower_bound(ps_.begin(), ps_.end(), tl, cmpobj_);
		auto upper = std::upper_bound(lower, ps_.end(), br, cmpobj_);
		mw.nparticles = 0;
		Particle newmean(Particle::Zero());
		for (auto iter = lower; iter < upper; iter++) {
			Particle& p(*iter);
			if (p(1) >= tl(1) && p(1) <= br(1)) {
				++mw.nparticles;
				newmean += p;
			}
		}
		bool ret = true;

		// Update center and radius accordingly
		if (mw.nparticles) {
			newmean /= mw.nparticles;
			if (abs(mw.center(0) - newmean(0))
				+ abs(mw.center(1) - newmean(1)) < 5.0)
				ret = false;
			mw.center = newmean;
			if (mw.nparticles > 0.25 * ps_.size())
				mw.radius *= 0.75;
		} else {
			mw.radius *= 1.25;
		}
		return ret;
	}

private:
	CompareParticle2D<Particle> cmpobj_;
};
}

#endif
