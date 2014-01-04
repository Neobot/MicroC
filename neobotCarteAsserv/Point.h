#ifndef POINT_H
#define POINT_H

class Point 
{
public:
    enum TypeAsserv
    {
        Aucun = 0,
        EnAvant = 1,
        EnArriere = 2,
        Auto = 13,
    };
    
    enum TypeDeplacement
    {
        TournePuisAvance = 0,
        TourneEtAvance = 1,
        TourneSeulement = 2,
        AvanceSeulement = 3,
    };

    Point();

    float x;
    float y;
    float theta;
    bool pointArret;
    TypeAsserv typeAsserv;
    TypeDeplacement typeDeplacement;
    float vitessMax;
};

#endif // POINT_H
