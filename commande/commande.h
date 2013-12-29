#ifndef COMMANDE_H
#define COMMANDE_H

/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Neobot wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy us a beer in return.
 * ----------------------------------------------------------------------------
 */

#include "Robot.h"

class Commande
{
    public:
    
        enum TypeDeplacement
        {
            TournePuisAvance = 0,
            TourneEtAvance = 1,
            TourneSeulement = 2,
            AvanceSeulement = 3,
        };
        
        enum EtatAsserv
        {
            Actif = 0,
            Desactive = 1,
        };
        
        enum SensDeplacement
        {
            avant = 0,
            Arriere = 1,
        };
        
        TypeDeplacement _typeDeplacementCourant;
        EtatAsserv _etatAsservCourant;
        SensDeplacement _sensDeplacementCourant;
}

#endif
