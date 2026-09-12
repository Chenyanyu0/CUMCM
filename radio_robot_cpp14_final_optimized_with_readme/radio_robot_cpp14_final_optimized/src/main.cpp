
#include "api.h"
#include "search.h"
#include "localization.h"
#include "planner.h"
#include "logger.h"
#include <map>

int main(){

    API api;

    if(!api.enter())
        return 0;

    std::map<int,bool> found;
    std::vector<Target> targets;

    for(auto p:searchPoints()){
        for(int ch=1;ch<=20;ch++){

            if(found[ch])
                continue;

            auto r=api.measure(p,ch);

            if(r["measure_result"]=="direction"){
                found[ch]=true;

                targets.push_back({
                    ch,
                    p,
                    (double)r["svd_deg"],
                    p,
                    {},
                    {{0,0},0},
                    false
                });
            }
        }
    }

    auto route=optimizeRoute(targets);

    for(int id:route){
        if(!targets[id].cleared){
            if(localizeAndClear(
                targets[id].channel,
                targets[id].firstPoint,
                targets[id].firstAngle))
                targets[id].cleared=true;
        }
    }

    saveLog();
    api.exit();
    return 0;
}
