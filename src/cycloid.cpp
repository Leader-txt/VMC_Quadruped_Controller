#include "cycloid.h"
using namespace std;

Cycloid::Cycloid(){

}
Cycloid::Cycloid(float length,float height,float flightPercent,float bodyHeight):
            Length(length),Height(height),FlightPercent(flightPercent),BodyHeight(bodyHeight){}
CycloidResult Cycloid::generate(float t){
    CycloidResult res;
    float intpart;
    t = modf(t,&intpart);
    res.isFlightPercent = true;
    if (t < FlightPercent){
        float Tm = FlightPercent;
        res.x = Length*(t/Tm-1/(2*M_PI)*sin(2*M_PI*t/Tm));
        res.y = Height*(((t < Tm/2) ? 1 : -1) *
                    (2*(t/Tm-1/(4*M_PI)*sin(4*M_PI*t/Tm))-1)+1);
    }
    else{
        res.isFlightPercent = false;
        float Tm = 1-FlightPercent;
        t -= FlightPercent;
        res.x = Length*((2*Tm-t)/Tm+1/(2*M_PI)*sin(2*M_PI*t/Tm)-1);
        res.y = 0;
    }
    res.x = -res.x + Length/2;
    res.y = BodyHeight - res.y;
    return res;
}