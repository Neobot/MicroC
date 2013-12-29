#include "Trajectoire.h"
#include "Cercle.h"
#include "Droite.h"
#include "MPoint.h"

Trajectoire::Trajectoire()
{

    Point _Probot;
    _ang = 0.0;
    _dist = 0.0;
    
    _estArrive = true;;
    _finis = true;
        
    _typeTraj = Aucune;
    
    _droite = new Droite();
    _cercle = new Cercle();
    _cercleArrive = new Cercle();
    _cercleArrive->_r = DIST_ARRIVE;
}

void Trajectoire::calculAD(float d, float sens)
{
    MPoint *nextPts;
    MPoint *pos = _Probot->getMPoint();
    
    if (!_estArrive)
    {
        if (_typeTraj == Aucune)
        {
            nextPts = _Pcible->getMPoint();
        }
        else if (_typeTraj == Droite)
        {
            nextPts = _droite->DepalcePts(_Pcible->getMPoint(), d, sens);
        }
        else if (_typeTraj == Cercle)
        {
            nextPts = _cercle->DepalcePts(_Pcible->getMPoint(), d, sens);
        }
        else
        {
            nextPts = Pos;
        }
        
        float dx = nextPts->_x - pos->_x;
        float dy = nextPts->_y - pos->_y;
        _ang = atan2(dy,dx) - _Probot->theta;
        _dist = sqrt(dx * dx + dy * dy); 
        
    }
    else
    {
        _ang = 0.0;
        _dist = 0.0;
    }
}

void Trajectoire::nextPts()
{
    if (!_trajPts.isEmpty())
    {
        _Pcible = _traj.pop();
        
        if (!_trajType.isEmpty())
        {
            _typeTraj = _trajType.pop();
        }
        else
        {
            _typeTraj = Aucune;
        }
        
        _cercleArrive->xc = _Pcible->x;
        _cercleArrive->yc = _Pcible->y;

    }
    else
    {
        _finis = true;
    }
}

void Trajectoire::Finis()
{
    _trajPts.clear();
    _trajType.clear();
    _finis = true;
    _estArrive = true;
}

void Trajectoire::ajoutPoint(Point point, TypeTraj type)
{ 
    _trajPts.push(point);
    _trajType.push(type);
}

bool Trajectoire::estArrive()
{ 
    MPoint *p = _Probot->getMPoint();
   _estArrive = _cercleArrive->dansCercle(p);
   
   return _estArrive;
}