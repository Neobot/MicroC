#ifndef CONSIGNE_H
#define CONSIGNE_H

#include "Parameters.h"

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

const float COMMANDE_MOTEUR_MAX = 65535.0;
const float DIST_ARRIVE_AVEC_FREINAGE = 5.0;
const float DIST_ARRIVE_SANS_FREINAGE = 5.0;

class Consigne
{
    public:
	  Consigne(float vmax, float amax, float periodeConsigne, float dccCoeff, float distArrive = DIST_ARRIVE_AVEC_FREINAGE); // 5 mm d'arrivée
		
      float calculConsigne(float deltaDistRealise);
      void setDemande(float dist, bool freinage = true);
      void setVmaxParcourt(float vmax);
      void setAmaxParcourt(float amax);
      void majDistAccDcc();
      void majConsigneMax();
      void majVariationConsigneMax();
      float transformeDeltaDistanceEnConsigne(float delta);
      bool estArrive();
      bool calcEstArrive();
      
      float _consigne;
      
      float _distArrive;
      
      enum Phase{Transitoire = 0, Stationaire = 1, Arrive = 2};
      
      float _vitessMax;
      float _vitessMaxParcourt;
      float _vitessCourrante;
      float _accelerationMaxParcourt;
      float _periodeMajConsigne;
      
      float _consignePrec;
      
      float _distRealise;
      
      float _distDemande;
      
      float _sens;
      
      float _distAcc;
      float _distDcc;
      bool doitFreiner;

      float _consigneMax;
      float _variationConsigneMax;
          
      Phase _phase;
      bool _acceleration;
      
      float _dccCoeff;
};

#endif // CONSIGNE_H

