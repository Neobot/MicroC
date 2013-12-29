#include "Trapeze.h"


Trapeze::Trapeze()
{
    _derive = 0.0;

    _prevVal = 0.0;

    _limitDevPos = 0.0;
    
    _limitAcePos = 0.0;
    _limitAceNeg = 0.0;
}

void Trapeze::setLimitVit(float vit)
{
    _limitDevPos = vit;
}

void Trapeze::setLimitAcc(float accPos, float accNeg)
{
    _limitAcePos = accPos;
    _limitAceNeg = accNeg;
}

float Trapeze::filter(float distRest, float vMaxSuivante)
{
    float acc;
    float out;
    float dFreinage;

    // t cst => vitesse equivalente à la distance
    _derive = distRest; 
    
    // limitation de la vitesse maximum
    if (_derive > _limitDevPos)
    {
        _derive = _limitDevPos;
    }
    
    // Freinage v²/2a (si on passe de v à v0 != 0 (v² - v0²)/2a)
    dFreinage = _prevVal * _prevVal / (2.0 * _limitAceNeg);
    
    if (vMaxSuivante >=0)
        dFreinage -= (vMaxSuivante * vMaxSuivante) / (2.0 * _limitAceNeg);
    
    if (dFreinage >= distRest)
    {
        _derive = _prevVal - _limitAceNeg;
    }
    
    acc = _derive - _prevVal;
    
    // limitation de l'acceleration maximum
    if (acc > _limitAcePos)
    {
        acc = _limitAcePos;
    }
    else if (acc < -_limitAceNeg)
    {
        acc = -_limitAceNeg;
    }
    
    out = _prevVal + acc;
    _prevVal = out;

    return out;
}