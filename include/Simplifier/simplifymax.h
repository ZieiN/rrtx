
#ifndef OMPL_GEOMETRIC_PATH_SIMPLIFIER_
#define OMPL_GEOMETRIC_PATH_SIMPLIFIER_

#include <limits>
#include <vector>
#include "RandomNumbers.h"
#include "../point.h"
#include "../map.h"

using namespace std;
class PathSimplifier {
public:

    PathSimplifier(Map &mp):mp_(mp){}

    ~PathSimplifier() = default;

    bool reduceVertices(vector<Point> &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0,
                        double rangeRatio = 0.33);

    bool shortcutPath(vector<Point> &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0,
                      double rangeRatio = 0.33, double snapToVertex = 0.005);

    bool collapseCloseVertices(vector<Point> &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0);

    void smoothBSpline(vector<Point> &path, unsigned int maxSteps = 5,
                       double minChange = std::numeric_limits<double>::epsilon());


    bool simplifyMax(vector<Point> &path);

protected:


    /** \brief Flag indicating whether the states removed from a motion should be freed */
//    bool freeStates_;

    /** \brief Instance of random number generator */
    ompl::RNG rng_;

    double length(vector<Point> &path);
    Map mp_;
    bool checkMotion(Point &p1, Point &p2);
    bool checkDubinsPath(Dubins &d, DubinsPath &dp, Point *p0, double stx, double sty, double ndx, double ndy, double t1, double t2);
    double distance(Point &p1, Point &p2);

    void copyState(Point *to, Point *from);

    Point cloneState(Point *pPoint);

    bool isValid(Point &point);

    void subdivide(vector<Point> &path);
};

#endif
