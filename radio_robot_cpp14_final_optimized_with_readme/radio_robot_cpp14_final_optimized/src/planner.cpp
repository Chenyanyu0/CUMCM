
#include "planner.h"
#include <cmath>
#include <algorithm>

static double dist(Point a,Point b){
    return hypot(a.x-b.x,a.y-b.y);
}

static double routeLength(
    const std::vector<int>& r,
    const std::vector<Target>& t)
{
    double s=0;
    for(int i=1;i<(int)r.size();i++)
        s+=dist(t[r[i-1]].estimate,t[r[i]].estimate);
    return s;
}

std::vector<int> optimizeRoute(std::vector<Target>& t)
{
    std::vector<int> r;

    for(int i=0;i<(int)t.size();i++)
        if(!t[i].cleared)
            r.push_back(i);

    if(r.size()<3)
        return r;

    // nearest neighbor initialization
    std::vector<int> ans;
    std::vector<int> used(t.size());

    int cur=r[0];
    ans.push_back(cur);
    used[cur]=1;

    while(ans.size()<r.size()){
        int best=-1;
        double d=1e18;
        for(int x:r){
            if(!used[x]){
                double nd=dist(
                    t[cur].estimate,
                    t[x].estimate);
                if(nd<d){
                    d=nd;
                    best=x;
                }
            }
        }
        cur=best;
        used[cur]=1;
        ans.push_back(cur);
    }

    // simple 2-opt
    bool improve=true;
    while(improve){
        improve=false;
        for(int i=1;i+2<(int)ans.size();i++){
            for(int j=i+1;j+1<(int)ans.size();j++){
                auto old=routeLength(ans,t);
                std::reverse(ans.begin()+i,
                             ans.begin()+j+1);
                auto now=routeLength(ans,t);
                if(now<old)
                    improve=true;
                else
                    std::reverse(ans.begin()+i,
                                 ans.begin()+j+1);
            }
        }
    }

    return ans;
}
