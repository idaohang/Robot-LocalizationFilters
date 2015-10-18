#include "ParticleFilter.h"
#include "PFHelper.h"
#include "MeanShift.h"
#include <cmath>
#if QT_VERSION >= 0x050000
#include <QtWidgets/QWidget>
#include <QtWidgets/QApplication>
#else
#include <QtGui/QWidget>
#include <QtGui/QApplication>
#endif
#include <QtGui/QKeyEvent>
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>

#define VERBOSE 0

#define MPARTICLE 	200
#define WALL		50.0
#define MAXRANGE	100.0

using namespace filter;

// 1D PF
typedef ParticleFilter<MPARTICLE, 1, 1, 1> PF;
typedef typename PF::Motion Motion;
typedef typename PF::Particle Particle;
typedef typename PF::Ob Ob;

//std::random_device g_rd;
//std::mt19937 g_gen(g_rd());
std::mt19937 g_gen(77);

class MM {
	std::normal_distribution<> dist_;
public:
	MM()
		:dist_(0.0, 1.0)
	{
	}
	// Given robot's motion u and particle in the previous previous frame
	// Return the anticipated location of this particle in current frame
	Particle operator()(const Motion& u, const Particle& x)
	{
		Particle rmove;
	        rmove << dist_(g_gen);
		return x + u + rmove;
	}
};

class ObM {
	static constexpr double theta = 10.0;
	static constexpr double theta2 = theta*theta;
	double frac_ = 1/(theta*sqrt(2*M_PI));
	double true_loc = WALL;
public:
	// Given robot's observation z, and (assumed) ground truth x
	// Return p(z|x)
	double operator()(const Ob& z, const Particle& x)
	{
		double truth = true_loc - x(0,0);
		double err = truth - z(0,0);
		double e = -(err*err)/(2*theta2);
#if VERBOSE
		printf("\terr: %f\te: %f\t\t\t", err, e);
#endif
		return frac_ * exp(e);
	}
};

Particle rpg()
{
	static std::uniform_real_distribution<> dis(0.0, MAXRANGE);
	Particle ret;
	ret << dis(g_gen);
	return ret;
}


class SimWidget : public QWidget {
	double loc_ = 0.0;
	double motion_ = 0.0;
	double motion_factor_ = 2.0;
	double wall_ = WALL;
	std::normal_distribution<> dist_;
	std::uniform_real_distribution<> udist_;
	PF& pf_;
	bool newmean_ = true;
	MeanShift<Particle, ClimbFunctor1D<Particle>> ms_;
	std::vector<MeanWindow<Particle>> mw_;
	MM mm_;
	ObM obm_;
public:
	SimWidget(PF& pf)
		:dist_(0, 0.7),
		pf_(pf),
		ms_(10, &rpg, CompareParticle1D<Particle>(), 20.0)
	{
	}
protected:
	void keyPressEvent(QKeyEvent* event)
	{
		bool next = false;
		bool ms = false;
		motion_ = 0.0;

		QWidget::keyPressEvent(event);
		int key = event->key();
		switch (key) {
			case Qt::Key_A:
			case Qt::Key_Left:
				motion_ = -1.0;
				next = true;
				break;
			case Qt::Key_D:
			case Qt::Key_Right:
				motion_ = 1.0;
			case Qt::Key_W:
			case Qt::Key_S:
			case Qt::Key_Up:
			case Qt::Key_Down:
				next = true;
				break;
			case Qt::Key_M:
				ms = true;
				break;
		}
		if (motion_ != 0.0)
			loc_ += motion_ * motion_factor_ + dist_(g_gen);
		if (next) {
			newmean_ = true;
			nextframe();
			update();
		}
		if (ms) {
			calc_ms();
			update();
		}
	}

	void calc_ms()
	{
		if (newmean_) {
			ms_.init_means(&(pf_.get_particles()), true);
			newmean_ = false;
		}
		ms_.meanshift_one();
		mw_ = ms_.get_current_window();
		printf("Current MW\n");
		for (const auto& mw: mw_) {
			printf("\tcenter %f, points %lu, radius: %f\n",
					mw.center(0),
					mw.nparticles,
					mw.radius);
		}
	}

	void nextframe()
	{
		std::normal_distribution<> obdist(wall_ - loc_, 5.0);
		Ob ob;
		ob << obdist(g_gen);
		if (motion_ != 0.0) {
			Motion m;
			m << motion_ * motion_factor_;
			pf_.filter(m, ob, mm_, obm_);
		} else {
			pf_.begin_frame();
			pf_.feed_stall_motion();
			pf_.feed_sensor(ob, obm_);
			pf_.end_frame();
		}
	}

	void paintEvent(QPaintEvent* event)
	{
		//QWidget::paintEvent(event);
		QPainter painter;
		painter.begin(this);

		std::vector<size_t> npix(width(), 0);
		// Fill the background
		auto background = QBrush(QColor(0, 0, 128));
		painter.fillRect(0, 0, width(), height(), background);
		for(const auto& particle : pf_.get_particles()) {
			size_t x = width() * (particle(0,0) / MAXRANGE);
#if VERBOSE
			printf("%.3f   ", particle(0,0));
#endif
			if (x >= 0 && x < npix.size())
				npix[x]++;
		}
		//printf("\n");
		painter.setPen(QColor(128,0,0));
		for(size_t i = 0; i < npix.size(); i++) {
			if (npix[i] > 0)
				painter.drawLine(i, height()/2, i, height()/2+npix[i]);
		}
#if VERBOSE
		printf("\nLoc %f\n", loc_);
#endif
		int iloc = loc_/MAXRANGE * width();
		painter.setPen(QColor(255,255,255));
		painter.drawLine(iloc, height()/2, iloc, height()/4);
		int iwall = wall_/MAXRANGE * width();
		painter.setPen(QColor(0,0,0));
		painter.drawLine(iwall, height()/2, iwall, 0);
		
		if (!mw_.empty()) {
			painter.setPen(QColor(255,255,255));
			int mw_counter = 1;
			for (const auto& mw : mw_) {
				int center = mw.center(0)/MAXRANGE * width();
				int w = mw.radius/MAXRANGE * width();
				painter.drawLine(
						center - w,
						height()/2 - 10*mw_counter,
						center + w,
						height()/2 - 10*mw_counter
						);
				mw_counter++;
			}
		}
	}
};

int main(int argc, char* argv[])
{
	QApplication app(argc, argv);
	PF pf;
	Motion m;
	m << -5.0;
	Ob ob;
	ob << 50.0;
	pf.filter(m, ob, MM(), ObM());
	pf.set_ejection(0.005, 0.99, &rpg);
	SimWidget widget(pf);
	widget.show();
	return app.exec();
}
