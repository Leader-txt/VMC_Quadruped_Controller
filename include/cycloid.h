#include <cmath>
#define _USE_MATH_DEFINES 

typedef struct{
    float x,y;
    bool isFlightPercent;
} CycloidResult;

class Cycloid{
    public: 
        float Length,Height,FlightPercent,BodyHeight;
        Cycloid();
        Cycloid(float length,float height,float flightPercent,float bodyHeight);
        CycloidResult generate(float t);
};