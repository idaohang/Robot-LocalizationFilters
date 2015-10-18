#ifndef MEAN_SHIFT_H
#define MEAN_SHIFT_H

#include <functional>
#include <eigen3/Eigen/Core>
#include "MeanWindow.h"

namespace filter {

template<typename Particle,
	typename ClimbFunctor>
class MeanShift {
public:
	typedef std::vector<Particle> ParticleSet;
	typedef std::function<bool (const Particle&, const Particle&)> PCmpFunctor;
	typedef Particle (*RandomParticleGenerator)();
	typedef MeanWindow<Particle> MW;

private:
	RandomParticleGenerator meanrpg_ = nullptr;
	std::vector<MW> mw_;
	int nmeans_ = 0;
	double init_radius_;
	PCmpFunctor pcmpf_;
	ParticleSet *ps_ = nullptr;

	MeanShift() = delete;

public:
	MeanShift(int ntrack, RandomParticleGenerator rpg, PCmpFunctor cmp, double rad)
	{
		set_meanshift(ntrack, rpg, cmp, rad);
	}

	void set_meanshift(int ntrack,
			RandomParticleGenerator rpg,
			PCmpFunctor cmp,
			double rad
			)
	{
		nmeans_ = ntrack;
		meanrpg_ = rpg;
		pcmpf_ = cmp;
		init_radius_ = rad;
	}

	/* Call for debugging purpose only */
	void init_means(ParticleSet* ps, bool reset)
	{
		ps_ = ps;
		if (reset || mw_.empty()) {
			mw_.clear();
			for (int i = 0; i < nmeans_; i++)
				mw_.emplace_back(MW({(*meanrpg_)(), 0, init_radius_}));
		} else {
			for (auto& mw : mw_)
				mw.radius = init_radius_;
		}
		std::sort(ps_->begin(), ps_->end(), pcmpf_);
	}

	void meanshift(ParticleSet* ps, bool reset = true)
	{
		init_means(ps, reset);

		for (auto& mw : mw_)
			mw_.emplace_back(climb(mw));
	}

	void meanshift_one()
	{
		ClimbFunctor cf(*ps_);
		for (auto& mw : mw_) {
			cf(mw);
		}
	}

	const std::vector<MW>& get_current_window() const
	{
		return mw_;
	}

	std::vector<MW>& get_current_window()
	{
		return mw_;
	}

private:

	MW climb(MW& mw)
	{
		ClimbFunctor cf(*ps_);
		bool changed;
		while (changed = cf(mw))
			;
		return mw;
	}
};

}

#endif
