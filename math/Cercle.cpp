#include "Cercle.h"
#include "Droite.h"
#include "MPoint.h"

Cercle::Cercle() : _xc(0.0), _yc(0.0), _r(0.0) {}

void Cercle::fromTwoPts(MPoint* P1, MPoint* P2)
{
    /* On consid�re que les 2 points sont le rayon */
    
    float dx = P2->_x - P1->_x;
    float dy = P2->_y - P1->_y;
    
    _xc = P1->_x + dx / 2.0;
    _yc = P1->_y + dx / 2.0
    _r = sqrt(dx * dx + dy * dy);
}

// attention peut �tre bizzare
void Cercle::fromTwoPtsAndR(MPoint* P1, MPoint* P2, float r, bool versLeHaut)
{
    /* on va changer 2 fois de rep�re 
     * on va se placer comme si c etait en le centre du cercle que l'on cherche
     * puis on va rechanger pour avoir les vrais coordon�es :)
     * il y a 2 centres possible d'ou la variable vers "versLeHaut";
    */
    
    _r = r;
    
    float dx = P2->_x - P1->_x;
    float dy = P2->_y - P1->_y;
    float d = sqrt(dx * dx + dy * dy); // distance entre les 2 points
    
    // dans un rep�re polaire l'angle du point le plus "� droite" est pi/2 - arcsin((d/2) / r).
    // et on le retourne pour le changement de repaire <= � verifier
    
    float theta = PI / 2.0 - arcsin(d / (2.0 * r)) - atan2(dy, dx);
    
    if (versLeHaut)
    {
        if (dx > 0)
        {
            _xc = P1->_x - r * cos(theta);
            _yc = P1->_y - r * sin(theta);
        }
        else
        {
            _xc = P2->_x - r * cos(theta);
            _yc = P2->_y - r * sin(theta);
        }
    }
    else
    {
        if (dx > 0)
        {
            _xc = P2->_x - r * cos(theta);
            _yc = P2->_y - r * sin(theta);
        }
        else
        {
            _xc = P1->_x - r * cos(theta);
            _yc = P1->_y - r * sin(theta);
        }
    }
}

bool Cercle::surCercle(MPoint* P)
{
    float dx = P->_x - _xc;
    float dy = P->_y - _yc;

    float r = dx * dx + dy * dy;

    return r == (_r *_r);
}

bool Cercle::dansCercle(MPoint* P)
{
    float dx = P->_x - _xc;
    float dy = P->_y - _yc;

    float r = dx * dx + dy * dy;

    return r <= (_r *_r);
}

MPoint* Cercle::project(MPoint* P)
{
    /* on est sur une projection passant par le centre du cercle
     * donc en prenant le centre du cercle comme ref d'un rep�re polaire ca se calcule tout seul
     * r = R du cercle
     * tetha = atan2(dy, dx) 
     * x = R cos(atan2(dy, dx))
     * y = R sin(atan2(dy, dx))
     */

    MPoint *proj = new MPoint();
    
    float dy = P->_y - _yc;
    float dx = P->_x - _xc;
    if (dx != 0.0 && dy != 0.0)
    {
        float theta = atan2(dy, dx);
        
        proj->_x = _r * cos(theta) + _xc;
        proj->_y = _r * sin(theta) + _yc;
    }
    
    return proj;
}

MPoint* Cercle::DepalcePts(MPoint* P, float d, float sens) // 1 = trigo, -1 antitrigo
{
    MPoint *Pp = project(P);
    MPoint *Ps = new MPoint(); // pts suivant
    
    // d = theta * r => theta = d / r
    // angle du point projet� dans le rep�re centtr� sur le cercle cercle
    float thetaPs = atan2(Pp->_y - _yc, Pp->_x - _xc) + sens * d / _r;
    
    Ps->_x = _r * cos(thetaPn) + _xc;
    Ps->_y = _r * sin(thetaPn) + _yc;
    
    return Ps;
}


