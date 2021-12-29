//
// Created by zain on 06.12.2021.
//

#include <map>
#include <iostream>
#include <fstream>
#include "../../include/Simplifier/simplifymax.h"
#include "../../include/Dubins/dubins.h"

bool PathSimplifier::reduceVertices(vector<Point> &path, unsigned int maxSteps, unsigned int maxEmptySteps,
                                    double rangeRatio) {
    if (path.size() < 3)
        return false;

    if (maxSteps == 0)
        maxSteps = path.size();

    if (maxEmptySteps == 0)
        maxEmptySteps = path.size();

    bool result = false;
    unsigned int nochange = 0;

    if (checkMotion(path.front(), path.back()))
    {
        auto tmp = path.back();
        while(path.size()>1){
            path.pop_back();
        }
        path.push_back(tmp);
        result = true;
    }
    else
        for (unsigned int i = 0; i < maxSteps && nochange < maxEmptySteps; ++i, ++nochange)
        {
            int count = path.size();
            int maxN = count - 1;
            int range = 1 + (int)(floor(0.5 + (double)count * rangeRatio));

            int p1 = rng_.uniformInt(0, maxN);
            int p2 = rng_.uniformInt(std::max(p1 - range, 0), std::min(maxN, p1 + range));
            if (abs(p1 - p2) < 2)
            {
                if (p1 < maxN - 1)
                    p2 = p1 + 2;
                else if (p1 > 1)
                    p2 = p1 - 2;
                else
                    continue;
            }

            if (p1 > p2)
                std::swap(p1, p2);

            if (checkMotion(path[p1], path[p2]))
            {
                path.erase(path.begin() + p1 + 1, path.begin() + p2);
                nochange = 0;
                result = true;
            }
        }
    return result;
}

bool
PathSimplifier::shortcutPath(vector<Point> &path, unsigned int maxSteps, unsigned int maxEmptySteps, double rangeRatio,
                             double snapToVertex) {

//    for(int i=0; i+1<path.size(); ++i) {
//        auto it = path[i], it1 = path[i + 1];
//        assert(checkMotion(it, it1));
//    }
    if (path.size() < 3)
        return false;

    if (maxSteps == 0)
        maxSteps = path.size();

    if (maxEmptySteps == 0)
        maxEmptySteps = path.size();

    // costs[i] contains the cumulative cost of the path up to and including state i
//    std::vector<base::Cost> costs(path.size(), obj_->identityCost());
    std::vector<double> dists(path.size(), 0.0);
    for (unsigned int i = 1; i < dists.size(); ++i)
    {
//        costs[i] = obj_->combineCosts(costs[i - 1], obj_->motionCost(path[i - 1], path[i]));
        dists[i] = dists[i - 1] + distance(path[i - 1], path[i]);
    }
    // Sampled path closer than 'threshold' distance to any existing state in the path
    // are snapped to the close state
    double threshold = dists.back() * snapToVertex;
    // The range (distance) of a single connection that will be attempted
    double rd = rangeRatio * dists.back();

    Point *temp0 = new Point;
    Point *temp1 = new Point;
    bool result = false;
    unsigned int nochange = 0;
    // Attempt shortcutting maxSteps times or when no improvement is found after
    // maxEmptySteps attempts, whichever comes first
    for (unsigned int i = 0; i < maxSteps && nochange < maxEmptySteps; ++i, ++nochange)
    {
        // Sample a random point anywhere along the path
        Point *s0 = new Point;
        int index0 = -1;
        double t0 = 0.0;
        double distTo0 = rng_.uniformReal(0.0, dists.back());  // sample a random point (p0) along the path
        auto pit =
                std::lower_bound(dists.begin(), dists.end(), distTo0);  // find the NEXT waypoint after the random point
        int pos0 = pit == dists.end() ? dists.size() - 1 :
                   pit - dists.begin();  // get the index of the NEXT waypoint after the point

        if (pos0 == 0 || dists[pos0] - distTo0 < threshold)  // snap to the NEXT waypoint
            index0 = pos0;
        else
        {
            while (pos0 > 0 && distTo0 < dists[pos0])
                --pos0;
            if (distTo0 - dists[pos0] < threshold)  // snap to the PREVIOUS waypoint
                index0 = pos0;
        }

        // Sample a random point within rd distance of the previously sampled point
        Point *s1 = nullptr;
        int index1 = -1;
        double t1 = 0.0;
        double distTo1 =
                rng_.uniformReal(std::max(0.0, distTo0 - rd),
                                 std::min(distTo0 + rd, dists.back()));   // sample a random point (distTo1) near s0
        pit = std::lower_bound(dists.begin(), dists.end(), distTo1);  // find the NEXT waypoint after the random point
        int pos1 = pit == dists.end() ? dists.size() - 1 :
                   pit - dists.begin();  // get the index of the NEXT waypoint after the point

        if (pos1 == 0 || dists[pos1] - distTo1 < threshold)  // snap to the NEXT waypoint
            index1 = pos1;
        else
        {
            while (pos1 > 0 && distTo1 < dists[pos1])
                --pos1;
            if (distTo1 - dists[pos1] < threshold)  // snap to the PREVIOUS waypoint
                index1 = pos1;
        }

        // Don't waste time on points that are on the same path segment
        if (pos0 == pos1 || index0 == pos1 || index1 == pos0 || pos0 + 1 == index1 || pos1 + 1 == index0 ||
            (index0 >= 0 && index1 >= 0 && abs(index0 - index1) < 2))
            continue;

//        for(int i=0; i+1<path.size(); ++i){
//            assert(checkMotion(path[i], path[i+1]));
//        }
        // Get the state pointer for costTo0
        if (index0 >= 0)
        {
            s0 = &path[index0];
        }
        else
        {
//            assert(distTo0>dists[pos0] && distTo0<dists[pos0+1]);
            t0 = (distTo0 - dists[pos0]) / (dists[pos0 + 1] - dists[pos0]);
            Dubins d(MIN_RADIUS, 0);
            d.interpolate(&path[pos0], &path[pos0 + 1], t0, temp0);
//            cout<<pos0<<" "<<path.size()<<endl;
//            cout<<distance(path[pos0], *temp0)+ distance(*temp0, path[pos0+1])<<endl;
//            cout<<distance(path[pos0], path[pos0+1])<<endl;
//            cout<<path[pos0].x_<<" "<<path[pos0].y_<<" "<<path[pos0].theta_<<endl;
//            cout<<path[pos0+1].x_<<" "<<path[pos0+1].y_<<" "<<path[pos0+1].theta_<<endl;
//            cout<<temp0->x_<<" "<<temp0->y_<<" "<<temp0->theta_<<endl;
//            assert(checkMotion(path[pos0], path[pos0+1]));
//            assert(checkMotion(*temp0, path[pos0+1]));
//            assert(checkMotion(path[pos0], *temp0));
            s0 = temp0;
        }

//        for(int i=0; i+1<path.size(); ++i) {
//            auto it = path[i], it1 = path[i + 1];
//            assert(checkMotion(it, it1));
//        }
        // Get the state pointer for costTo1
        if (index1 >= 0)
        {
            s1 = &path[index1];
        }
        else
        {
//            assert(distTo1>dists[pos1] && distTo1<dists[pos1+1]);
            t1 = (distTo1 - dists[pos1]) / (dists[pos1 + 1] - dists[pos1]);
            Dubins d(MIN_RADIUS, 0);
            d.interpolate(&path[pos1], &path[pos1 + 1], t1, temp1);
//            cout<<pos1+1<<" "<<path.size()<<endl;
//            cout<<distance(path[pos1], *temp1)<<" "<<distance(*temp1, path[pos1+1])<<endl;
//            cout<<distance(path[pos1], path[pos1+1])<<endl;
//            cout<<path[pos1].x_<<" "<<path[pos1].y_<<" "<<path[pos1].theta_<<endl;
//            cout<<path[pos1+1].x_<<" "<<path[pos1+1].y_<<" "<<path[pos1+1].theta_<<endl;
//            cout<<temp1->x_<<" "<<temp1->y_<<" "<<temp1->theta_<<endl;
//            assert(checkMotion(path[pos1], path[pos1+1]));
//            if(checkMotion(*temp1, path[pos1+1])){
//
//            }
//            assert(checkMotion(*temp1, path[pos1+1]));
//            assert(checkMotion(path[pos1], *temp1));
            s1 = temp1;
        }

        // Check for validity between s0 and s1

//        for(int i=0; i+1<path.size(); ++i){
//            assert(checkMotion(path[i], path[i+1]));
//        }
        if (pos0 > pos1)
        {
            std::swap(pos0, pos1);
            std::swap(index0, index1);
            std::swap(s0, s1);
            std::swap(t0, t1);
        }
        if (checkMotion(*s0, *s1))
        {
//            for(int i=0; i+1<path.size(); ++i){
//                assert(checkMotion(path[i], path[i+1]));
//            }
            // Now that path are in the right order, make sure the cost actually decreases.
            double s0PartialCost = (index0 >= 0) ? 0 : distance(*s0, path[pos0 + 1]);
            double s1PartialCost = (index1 >= 0) ? 0 : distance(path[pos1], *s1);
            double alongPath = s0PartialCost;
            int posTemp = pos0 + 1;
            while (posTemp < pos1)
            {
                alongPath += distance(path[posTemp], path[posTemp + 1]);
                posTemp++;
            }
            alongPath += s1PartialCost;
            if (alongPath < distance(*s0, *s1))
            {
                // The cost along the path from state 0 to 1 is better than the straight line motion cost between the
                // two.
                continue;
            }
            // Otherwise, shortcut cost is better!

//            if(index0<0){
//                assert(checkMotion(path[pos0], *s0));
//            }
//            if(index1<0){
//                assert(checkMotion(*s1, path[pos1+1]));
//            }
            // Modify the path with the new, shorter result
            if (index0 < 0 && index1 < 0)
            {
//                assert(checkMotion(path[pos0], *s0));
//                assert(checkMotion(*s1, path[pos1+1]));
                if (pos0 + 1 == pos1)
                {
//                    assert(checkMotion(*s0, *s1));
                    copyState(&path[pos1], s0);
                    path.insert(path.begin() + pos1 + 1, cloneState(s1));
                }
                else
                {
//                    assert(checkMotion(*s0, *s1));
                    copyState(&path[pos0 + 1], s0);
                    copyState(&path[pos1], s1);
                    path.erase(path.begin() + pos0 + 2, path.begin() + pos1);
                }
            }
            else if (index0 >= 0 && index1 >= 0)
            {
//                assert(checkMotion(path[index0], path[index1]));
                path.erase(path.begin() + index0 + 1, path.begin() + index1);
            }
            else if (index0 < 0 && index1 >= 0)
            {
//                assert(checkMotion(*s0, path[index1]));
                copyState(&path[pos0 + 1], s0);
                path.erase(path.begin() + pos0 + 2, path.begin() + index1);
            }
            else if (index0 >= 0 && index1 < 0)
            {
//                assert(checkMotion(path[index0], *s1));
                copyState(&path[pos1], s1);
                path.erase(path.begin() + index0 + 1, path.begin() + pos1);
            }

            for(int i=0; i+1<path.size(); ++i) {
                auto it = path[i], it1 = path[i + 1];
//                assert(checkMotion(it, it1));
            }
            // fix the helper variables
            dists.resize(path.size(), 0.0);
//            costs.resize(path.size(), obj_->identityCost());
            for (unsigned int j = pos0 + 1; j < dists.size(); ++j)
            {
//                costs[j] = obj_->combineCosts(costs[j - 1], obj_->motionCost(path[j - 1], path[j]));
                dists[j] = dists[j - 1] + distance(path[j - 1], path[j]);
            }
            threshold = dists.back() * snapToVertex;
            rd = rangeRatio * dists.back();
            result = true;
            nochange = 0;
        }
    }
//    for(int i=0; i+1<path.size(); ++i) {
//        auto it = path[i], it1 = path[i + 1];
//        assert(checkMotion(it, it1));
//    }
    return result;
}

bool PathSimplifier::collapseCloseVertices(vector<Point> &path, unsigned int maxSteps, unsigned int maxEmptySteps) {
    if (path.size() < 3)
        return false;

    if (maxSteps == 0)
        maxSteps = path.size();

    if (maxEmptySteps == 0)
        maxEmptySteps = path.size();

    // compute pair-wise distances in path (construct only half the matrix)
    std::map<std::pair<const Point, const Point>, double> distances;
    for (unsigned int i = 0; i < path.size(); ++i)
        for (unsigned int j = i + 2; j < path.size(); ++j)
            distances[std::make_pair(path[i], path[j])] = distance(path[i], path[j]);

    bool result = false;
    unsigned int nochange = 0;
    for (unsigned int s = 0; s < maxSteps && nochange < maxEmptySteps; ++s, ++nochange)
    {
        // find closest pair of points
        double minDist = std::numeric_limits<double>::infinity();
        int p1 = -1;
        int p2 = -1;
        for (unsigned int i = 0; i < path.size(); ++i)
            for (unsigned int j = i + 2; j < path.size(); ++j)
            {
                double d = distances[std::make_pair(path[i], path[j])];
                if (d < minDist)
                {
                    minDist = d;
                    p1 = i;
                    p2 = j;
                }
            }

        if (p1 >= 0 && p2 >= 0)
        {
            if (checkMotion(path[p1], path[p2]))
            {
                path.erase(path.begin() + p1 + 1, path.begin() + p2);
                result = true;
                nochange = 0;
            }
            else
                distances[std::make_pair(path[p1], path[p2])] = std::numeric_limits<double>::infinity();
        }
        else
            break;
    }
    return result;
}

void PathSimplifier::smoothBSpline(vector<Point> &path, unsigned int maxSteps, double minChange) {
    if (path.size() < 3)
        return;

    Point *temp1 = new Point;
    Point *temp2 = new Point;

    for (unsigned int s = 0; s < maxSteps; ++s)
    {
        subdivide(path);

        unsigned int i = 2, u = 0, n1 = path.size() - 1;
        Dubins d(MIN_RADIUS, 0);
        while (i < n1)
        {
            if (isValid(path[i - 1]))
            {
                d.interpolate(&path[i - 1], &path[i], 0.5, temp1);
                d.interpolate(&path[i], &path[i + 1], 0.5, temp2);
                d.interpolate(temp1, temp2, 0.5, temp1);
                if (checkMotion(path[i - 1], *temp1) && checkMotion(*temp1, path[i + 1]))
                {
                    if (distance(path[i], *temp1) > minChange)
                    {
                        copyState(&path[i], temp1);
                        ++u;
                    }
                }
            }

            i += 2;
        }

        if (u == 0)
            break;
    }
}


bool PathSimplifier::simplifyMax(vector<Point> &path) {
    if (path.size() < 3)
        return true;

    bool tryMore = true, valid = true;
    while (tryMore) {
        bool metricTryMore = true;
        unsigned int times = 0;
        do {
            bool shortcut = shortcutPath(path);
            metricTryMore = shortcut;
        } while (++times <= 5 && metricTryMore);

        smoothBSpline(path, 3, length(path) / 100.0);

        // we always run this if the metric-space algorithms were run.  In non-metric spaces this does not work.
//                const std::pair<bool, bool> &p = path.checkAndRepair(magic::MAX_VALID_SAMPLE_ATTEMPTS);
//                if (!p.second)
//                {
//                    valid = false;
//                    OMPL_WARN("Solution path may slightly touch on an invalid region of the state space");
//                }
//                else if (!p.first)
//                    OMPL_DEBUG("The solution path was slightly touching on an invalid region of the state space, but "
//                               "it was "
//                               "successfully fixed.");

            tryMore = reduceVertices(path);

            collapseCloseVertices(path);

        // try to reduce vertices some more, if there is any point in doing so
        times = 0;
        while (tryMore && ++times <= 5)
            tryMore = reduceVertices(path);


//            // we always run this if the metric-space algorithms were run.  In non-metric spaces this does not work.
//            const std::pair<bool, bool> &p = path.checkAndRepair(magic::MAX_VALID_SAMPLE_ATTEMPTS);
//            if (!p.second)
//            {
//                valid = false;
//                OMPL_WARN("Solution path may slightly touch on an invalid region of the state space");
//            }
//            else if (!p.first)
//                OMPL_DEBUG("The solution path was slightly touching on an invalid region of the state space, but it "
//                           "was "
//                           "successfully fixed.");

    }
    return true;
//    return valid || path.check();
}

double PathSimplifier::length(vector<Point> &path) {
    Dubins d(MIN_RADIUS,0);
    double ans = 0;
    for(int i=0; i+1<path.size(); ++i){
        ans += d.distance(&path[i], &path[i+1]);
    }
    return ans;
}

bool PathSimplifier::checkMotion(Point &p1, Point &p2) {
    Dubins d(MIN_RADIUS, 0);
    auto path=d.dubins(&p1, &p2);

//    ofstream out;
//    out.open(string("../output/map1/Dynamics/outSol" + to_string(0) + "-" + to_string(0) +
//            ".txt").c_str());
//    for(int i=1; i<=100; ++i){
//        Point p;
//        d.interpolate(&p1, path, (double)i/100.0, &p);
//        out<<p.x_<<" "<<p.y_<<" "<<p.x_<<" "<<p.y_<<" "<<endl;
//    }
//    out.close();
//    cout<<p1.x_<<" "<<p1.y_<<" "<<p1.theta_<<"-->"<<p2.x_<<" "<<p2.y_<<" "<<p2.theta_<<endl;
    int numOfTimeSteps = distance(p1, p2)*3+1;
//    cout<<"Here"<<numOfTimeSteps<<endl;
    for(int i=1; i<=numOfTimeSteps; ++i){
        Point p;
        d.interpolate(&p1, path, (double)i/numOfTimeSteps, &p);
//        cout<<p.x_<<" "<<p.y_<<endl;
        if(mp_.cellIsObstacle(p.x_, p.y_)) {
            return false;
        }
    }
    return true;
}

double PathSimplifier::distance(Point &p1, Point &p2) {
    Dubins d(MIN_RADIUS, 0);
    return d.distance(&p1, &p2);
}

void PathSimplifier::copyState(Point *to, Point *from) {
    to->x_ = from->x_;
    to->y_ = from->y_;
    to->theta_ = from->theta_;
}

Point PathSimplifier::cloneState(Point *pPoint) {
    Point tmp;
    copyState(&tmp,pPoint);
    return  tmp;
}

bool PathSimplifier::isValid(Point &point) {
    return !mp_.cellIsObstacle(point.x_, point.y_);
}

void PathSimplifier::subdivide(vector<Point> &path) {
    if (path.size() < 2)
        return;
    std::vector<Point> newStates(1, path[0]);
    Dubins d(MIN_RADIUS, 0);
    for (unsigned int i = 1; i < path.size(); ++i)
    {
        Point *temp = new Point;
        d.interpolate(&newStates.back(), &path[i], 0.5, temp);
        newStates.push_back(*temp);
        newStates.push_back(path[i]);
    }
    path.swap(newStates);
}
