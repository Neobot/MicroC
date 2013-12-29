#include "Droite.h"
#include "MPoint.h"

Droite::Droite() : _a(0.0), _b(0.0), _c(0.0) {}

void Droite::fromTwoPts(MPoint* P1, MPoint* P2)
{
    float dx = P2->_x - P1->_x;
    
    if (dx != 0.0)
    {
        _a = dx / (P2->_y - P1->_y);
        _b = P2->_y - _a * P2->_x;
        _c = -1.0;
    }
    else
    {
        _a = 1.0;
        _b = -P2->_x;
        _c = 0.0;
    }

}

bool Droite::isOnLine(MPoint* P1)
{
    
    return 0.0 == (_a * P1->_x + _b * P1->_y + _c);
}

MPoint* Droite::intersec(Droite* D)
{
    MPoint *P = new MPoint();
    
    if (_c == D->_c)
    {
        if (_c == -1.0)
        {
            if (_a != D->_a)
            {
                P->_x = (_b - D->_b) / (D->_a - _a);
                P->_y = _a * P->_x + _b;
            }
            else
            {
                // Droite "//"
                return null;
            }
        }
        else
        {
            // Droite "//"
            return null;
        }
    
    }
    else
    {
        if (_c == 0.0)
        {
            P->_x = -_b;
            P->_y = D->_a * P->_x + D->_b;
        }
        else
        {
            P->_x = -D->_b;
            P->_y = _a * P->_x + _b;
        }
    }
    
    return P;
}

Droite* Droite::getPerpendiculaireDroite(MPoint* P)
{
    Droite *D = new Droite();
    if (_c == -1.0)
    {
        
        if (_a == 0.0)
        {
            D->_a = 1.0;
            D->_b = -P->_x;
            D->_c = 0.0;
        }
        else
        {
            D->_a = -1.0 / _a;
            D->_b = P->_y - D2->_a * P->_x;
            D->_c = -1.0;
        }
    }
    else
    {
        D->_a = 0.0;
        D->_b = P->_y;
        D->_c = -1.0;
    }
    
    return D;
}


MPoint* Droite::project(MPoint* P)
{
    Droite *D = getPerpendiculaireDroite(P);
    return intersec(D);
}

MPoint* Droite::DepalcePts(MPoint* P, float d, float sens) // 1 = suis la droite, -1 remonte la droite
{
    MPoint *Pp = project(P);
    MPoint *Pn = new MPoint();
    
    if (_a == 0.0) // horizontal
    {
        Pn->_x = Pp->_x + sens * d;
        Pn->_y = Pp->_y;
    }
    else
    {  
        Pn->_y = Pp->_y + sens * (_a * d) / (sqrt(_a * _a + _c * _c));
        Pn->_x = -1.0 * (_c * Pn->_y + _b) / _a ;
    }
    
    return Pn;
}

