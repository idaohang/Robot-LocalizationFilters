#include "ParticleFilter.h"
#include <cmath>
#include <QtWidgets/QWidget>
#include <QtGui/QKeyEvent>
#include <QtGui/QPaintEvent>
#include <QtGui/QPainter>
#include <QtWidgets/QApplication>

#define MPARTICLE 1000
#define WALL		50.0
#define MAXRANGE	100.0

using namespace filter;

// 1D PF
typedef ParticleFilter<MPARTICLE, 1, 1, 1> PF;
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

class SimWidget : public QWidget {
	double loc_ = 0.0;
	double wall_ = WALL;
	std::random_device rd_;
	std::mt19937 gen_;
	std::normal_distribution<> dist_;
	std::uniform_real_distribution<> udist_;
	PF& pf_;
public:
	SimWidget(PF& pf)
		:rd_(),
		gen_(rd_()),
		dist_(2.0, 0.7),
		pf_(pf)
	{
	}
protected:
	void keyPressEvent(QKeyEvent* event)
	{
		bool next = false;
		QWidget::keyPressEvent(event);
		int key = event->key();
		switch (key) {
			case Qt::Key_A:
			case Qt::Key_Left:
				loc_ -= dist_(gen_);
				next = true;
				break;
			case Qt::Key_D:
			case Qt::Key_Right:
				loc_ += dist_(gen_);
			case Qt::Key_W:
			case Qt::Key_S:
			case Qt::Key_Up:
			case Qt::Key_Down:
				next = true;
				break;
		}
		if (next) {
			nextframe();
			update();
		}
	}

	void nextframe()
	{
	}

	void paintEvent(QPaintEvent* event)
	{
		QWidget::paintEvent(event);
		QPainter painter;
		painter.begin(this);

		std::vector<size_t> npix(width(), 0);
		// Fill the background
		auto background = QBrush(QColor(0, 0, 128));
		painter.fillRect(0, 0, width(), height(), background);
		for(const auto& particle : pf_.get_particles()) {
			size_t x = width() * (particle(0,0) / MAXRANGE);
			printf("%.3f   ", particle(0,0));
			npix[x]++;
		}
		printf("\n");
		painter.setPen(QColor(128,0,0));
		for(size_t i = 0; i < npix.size(); i++) {
			if (npix[i] > 0)
				painter.drawLine(i, height()/2, i, height()/2+npix[i]);
		}

		painter.end();
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
	SimWidget widget(pf);
	widget.show();
	return app.exec();
}
