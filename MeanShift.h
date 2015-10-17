#ifndef MEAN_SHIFT_H
#define MEAN_SHIFT_H

#include <functional>
#include <eigen3/Eigen/Core>

namespace filter {
template<typename Particle,
	typename ClimbFunctor>
class MeanShift {
public:
	typedef std::vector<Particle> ParticleSet;
	typedef std::function<bool (const Particle&, const Particle&)> PCmpFunctor;
	typedef Particle (*RandomParticleGenerator)();

	struct MeanWindow {
		Particle center;
		size_t nparticles;
		double radius;
	};
private:
	RandomParticleGenerator meanrpg_ = nullptr;
	ParticleSet means_;
	std::vector<MeanWindow> mw_;
	int nmeans_ = 0;
	double init_radius_;
	PCmpFunctor pcmpf_;
	std::reference_wrapper<ParticleSet> ps_;

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

	void filter_particles(ParticleSet& ps)
	{
		ps_ = std::ref(ps);
		meanshift();
	}

private:
	void init_means()
	{
		for (int i = 0; i < nmeans_; i++)
			means_.emplace_back((*meanrpg_)());
		mw_.clear();
	}

	void meanshift()
	{
		init_means();
		std::sort(ps_.begin(), ps_.end(), pcmpf_);
		for (auto& meanv : means_)
			mw_.emplace_back(climb(meanv));
	}

	MeanWindow climb(const Particle& meanv)
	{
		MeanWindow mw;
		ClimbFunctor cf(ps_);
		mw.center = meanv;
		mw.radius = init_radius_;
		bool changed;
		while (changed = cf(mw))
			;
		return mw;
	}
};

}

#endif
