#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <eigen3/Eigen/Core>
#include <random>
#include <algorithm>

namespace filter {

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
	ParticleSet particles_, bar_particles_;
	//std::random_device rd_;
	std::mt19937 gen_;
	Weight w_, accumw_;
	double alpha_slow_, alpha_fast_;
	double w_slow_ = 0.0, w_fast_ = 0.0;
	bool enable_ejection_ = false;
	RandomParticleGenerator rpg_ = nullptr;

public:
	ParticleFilter()
		:particles_(MParticle),
		bar_particles_(MParticle),
		//rd_(),
		//gen_(rd_()),
		gen_(88),
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

	void begin_frame()
	{
		w_.assign(MParticle, 1.0); // Reset to unit probablistic
	}

	// Feed motion data
	template<typename MotionModelFunctor>
	void feed_motion(const Motion& u, MotionModelFunctor mov)
	{
		for(size_t i = 0; i < MParticle; i++)
            bar_particles_[i] = mov(u, particles_[i]);
    }

	void feed_stall_motion()
	{
		bar_particles_ = particles_;
	}

	// Feed sensor data
	// This can be called multiple times for different landmarks
	template<typename ObservationModelFunctor>
	void feed_sensor(const Ob& z, ObservationModelFunctor ob)
	{
		for(size_t i = 0; i < MParticle; i++) {
			w_[i] *= ob(z, bar_particles_[i]);
			//printf("OB: %f\n", w_[i]);
		}
	}

	void end_frame()
	{
		resample();
	}

	template<typename MotionModelFunctor,
		 typename ObservationModelFunctor>
	void filter(const Motion& u, const Ob& z, MotionModelFunctor mov, ObservationModelFunctor ob)
	{
		begin_frame();
		feed_motion(u, mov);
		feed_sensor(z, ob);
		end_frame();
	}

	ParticleSet& get_particles()
	{
		return particles_;
	}

private:
	void resample()
	{
		accumw_[0] = w_[0];
		for(size_t i = 1; i < MParticle; i++) {
			accumw_[i] = accumw_[i-1] + w_[i];
		}
		double sumw = accumw_.back();
		double w_ave = sumw / double(MParticle);
		double cell_size = sumw/MParticle;
		std::uniform_real_distribution<> dis(0.0, cell_size);
		//printf("Resample: sumw %f\n", sumw);

		auto iter = accumw_.begin();
		double sample = dis(gen_);
		for(size_t i = 0; i < MParticle; i++) {
			while (*iter < sample && iter + 1 != accumw_.end())
				++iter;
			size_t idx = iter - accumw_.begin();
			particles_[i] = bar_particles_[idx]; // Add \bar{X}_{idx} to X
			sample += cell_size;
		}

		if (enable_ejection_) {
			w_slow_ += alpha_slow_ * (w_ave - w_slow_);
			w_fast_ += alpha_fast_ * (w_ave - w_fast_);
			double ejection_prob = std::max(0.0, 1.0 - w_fast_/w_slow_);
			eject_random_particles(ejection_prob);
		}
	}

	void eject_random_particles(double ejection_prob)
	{
		std::uniform_real_distribution<> dis(0.0, 1.0);
		for(size_t i = 0; i < MParticle; i++) {
			if (dis(gen_) < ejection_prob)
				particles_[i] = (*rpg_)();
		}
	}

};

}
#endif
