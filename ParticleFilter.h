#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <eigen3/Eigen/Core>
#include <random>
#include <algorithm>

template<size_t MParticle,
	size_t NState,
	size_t NObservation,
	size_t NMotion,
	typename Element = double>
class ParticleFilter {
public:
	typedef Eigen::Matrix<Element, 1 , NState> Particle;
	//typedef Eigen::Matrix<Element, MParticle , NState> ParticleSet;
	typedef std::vector<Particle> ParticleSet;
	typedef Eigen::Matrix<Element, 1, NObservation> Ob;
	typedef Eigen::Matrix<Element, 1, NMotion> Motion;
	typedef std::vector<double> Weight;
	typedef Particle (*RandomParticleGenerator)();
private:
	ParticleSet particles_;
	std::random_device rd_;
	std::mt19937 gen_;
	Weight w_, accumw_;
	double alpha_slow_, alpha_fast_;
	double w_slow_ = 0.0, w_fast_ = 0.0;
	bool enable_ejection_ = false;
	RandomParticleGenerator rpg_ = nullptr;

public:
	ParticleFilter()
		:particles_(MParticle),
		rd_(), gen_(rd_()),
		w_(MParticle), accumw_(MParticle)
	{
	}

	void set_ejection(double alpha_slow, double alpha_fast, RandomParticleGenerator rpg)
	{
		alpha_slow_ = alpha_slow;
		alpha_fast_ = alpha_fast;
		enable_ejection_ = true;
		rpg_ = rpg;
	}

	void disable_ejection()
	{
		enable_ejection_ = false;
	}

	template<typename MotionModelFunctor,
		 typename ObservationModelFunctor>
	void filter(const Motion& u, const Ob& z, MotionModelFunctor mov, ObservationModelFunctor ob)
	{
		ParticleSet xbar(MParticle);

		xbar[0] = mov(u, particles_[0]);
		w_[0] = ob(z, xbar[0]);
		accumw_[0] = w_[0];

		for(size_t i = 1; i < MParticle; i++) {
			xbar[i] = mov(u, particles_[i]);
			w_[i] = ob(z, xbar[i]);
			accumw_[i] = accumw_[i-1] + w_[i];
		}
		double sumw = accumw_.back();
		double w_ave = sumw / double(MParticle);
		std::uniform_real_distribution<> dis(0.0, sumw);

		double ejection_prob;
		if (enable_ejection_) {
			w_slow_ += alpha_slow_ * (w_ave - w_slow_);
			w_fast_ += alpha_fast_ * (w_ave - w_fast_);
			ejection_prob = std::max(0.0, 1.0 - w_fast_/w_slow_);
		}

		for(size_t i = 0; i < MParticle; i++) {
			if (enable_ejection_ && dis(gen_) < ejection_prob) {
				particles_[i] = (*rpg_)();
				continue;
			}
			double sample = dis(gen_);
			auto iter = std::lower_bound(accumw_.begin(),
					accumw_.end(),
					sample);
			size_t idx = iter - accumw_.begin();
			particles_[i] = xbar[idx]; // Add \bar{X}_{idx} to X
		}
	}

	const ParticleSet& get_particles()
	{
		return particles_;
	}
};

#endif
