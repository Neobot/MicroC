/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Neobot wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy us a beer in return.
 * ----------------------------------------------------------------------------
 */
 
/*
 * Project : Neobot
 * Version : 0.42
 * Date : 30/12/2012
 * Author : Neobot
 */

#ifndef TRAJECTOIRE_H
#define TRAJECTOIRE_H

const float DIST_ARRIVE = 5.0;

class Trajectoire
{
    // distance : entre current et arrivé
    // angle : moyenne entre point projeté sur la courbe et l'angle pour rejoindre l'arrivé ou projeté en avance 

  public:
  
    enum TypeTraj
    {
        Aucune = 0,
        Droite = 1,
        Cercle = 2,
    };
  
    QueueList<Point> _trajPts;
    QueueList<TypeTraj> _trajType;
    Point _Pcible;
    Point _Probot;
    float _ang;
    float _dist;
    
    bool _estArrive;
    bool _finis;
        
    TypeTraj _typeTraj;
    
    Droite *_droite;
    Cercle *_cercle;

    Cercle *_cercleArrive;
}

#endif