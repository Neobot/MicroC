#include "Robot.h"

Robot::Robot(Servo* servoArG, Servo* servoArD, float periodAsserv, float x, float y, float theta)
{
  _tourneFini = false;
  _pingReceived = false;
  
  this->_typeDeplacement = TournePuisAvance;
  this->_typeAsserv = Auto;

	Point point;
	point.x = x;
	point.y = y;
	point.theta = theta;

        this->servoArG = servoArG;
        this->servoArD = servoArD;
	
	this->teleport(point);
	this->forceObjectif(point);

	this->_thetaTotal = 0.0;
	this->_deltaDist = 0.0;
  this->_deltaOrient = 0.0;
	this->_periodAsserv = periodAsserv;
  this->_commmandeRoueGauche = 0;
  this->_commmandeRoueDroite = 0;
	
	this->pasPrecendentGauche = 0.0;
	this->pasPrecendentDroit = 0.0;
  
  this->_sensAvantRoueGauche = true;
  this->_sensAvantRoueDroite = true;
	
	this->_pidDist = new PID(ACTIVE_PID_DISTANCE, KP_DISTANCE, KD_DISTANCE, KI_DISTANCE);
	this->_pidOrientation = new PID(ACTIVE_PID_ANGLE, KP_ANGLE, KD_ANGLE, KI_ANGLE);
	
	this->_consigneDist = new Consigne(VITESSE_MAX, ACCELARATION_MAX_EN_REEL_LIN, periodAsserv, 1.4, 5);
	this->_consigneOrientation = new Consigne(VITESSE_MAX_ROT, ACCELARATION_MAX_EN_REEL_ROT, periodAsserv, 0.05);
  
}

void Robot::teleport(Point point)
{
    this->position = point;
    this->flush();
    this->pointSuivant = this->position;
    
}

void Robot::forceObjectif(Point point)
{
    this->pointSuivant = point;
}

void Robot::ajoutPoint(Point point)
{ 
    this->queue.push(point);
}

void Robot::ajoutPoint(float x, float y, bool pointArret, int typeDeplacement, float vitessMax)
{ 
    Point point;
    point.x = x;
    point.y = y;
    point.pointArret = pointArret;
    point.vitessMax = vitessMax;
    point.typeDeplacement = (Point::TypeDeplacement) typeDeplacement;

    this->queue.push(point);
}

void Robot::flush()
{
    this->queue.clear();
}

void Robot::stop()
{
    this->flush();

    this->pointSuivant.x = this->position.x + this->_consigneDist->_distDcc * cos(this->position.theta);
    this->pointSuivant.y = this->position.y + this->_consigneDist->_distDcc * sin(this->position.theta);
}

void Robot::majPosition(float pasRoueGauche, float pasRoueDroite)
{
    float distGauche = ((float) COEF_CORRECTION_ROUE_FOLLE_GAUCHE) *  (pasRoueGauche - this->pasPrecendentGauche) * COEFF_CONVERTION_PAS_METRE;
    float distDroite = ((float) COEF_CORRECTION_ROUE_FOLLE_DROITE) *  (pasRoueDroite - this->pasPrecendentDroit) *  COEFF_CONVERTION_PAS_METRE;
    float k = 1.0;
     
    this->_deltaDist = (float)( distGauche + distDroite) / 2.0;
    this->_deltaOrient = (float) ( distDroite - distGauche ) / (float) ENTRAXE;
	
    if (this->_deltaOrient != 0.0)
    {
      k = 2.0 * sin (this->_deltaOrient / 2.0) / this->_deltaOrient ; //approx circulaire
    }

    float dX = k * this->_deltaDist * cos( this->position.theta + this->_deltaOrient / 2.0);
    float dY = k * this->_deltaDist * sin( this->position.theta + this->_deltaOrient / 2.0);
  
    float deriveX = CORFUGE * this->_deltaOrient * dY; 
    float deriveY = - CORFUGE * this->_deltaOrient * dX;
  
    this->position.x += dX + deriveX;
    this->position.y += dY + deriveY;
    this->position.theta += this->_deltaOrient;
	
    this->pasPrecendentGauche = pasRoueGauche;
    this->pasPrecendentDroit = pasRoueDroite;
}


void Robot::calculConsigne()
{
  float thetaDemande = this->_deltaOrient * ENTRAXE;

  if (this->_typeDeplacement == TourneEtAvance)
  {
    this->_consigneDist->calculConsigne(this->_deltaDist);
    this->_consigneOrientation->calculConsigne(thetaDemande);
  }
  else if (_typeDeplacement == TournePuisAvance)
  {
    if (this->_consigneOrientation->calcEstArrive() == false && !_tourneFini)
    {
      this->_consigneDist->_consigne = 0;
      this->_consigneOrientation->calculConsigne(thetaDemande);
    }
    else
    {
      if (!_tourneFini)
      {
        _tourneFini = true;
      }
      this->_consigneDist->_consigne = this->_consigneDist->calculConsigne(this->_deltaDist);
      this->_consigneOrientation->calculConsigne(thetaDemande);
    }
  }
  else if (_typeDeplacement == TourneSeulement)
  {
    this->_consigneDist->_consigne = 0;
    this->_consigneOrientation->calculConsigne(thetaDemande);
  }
  else if (_typeDeplacement == AvanceSeulement)
  {
    this->_consigneOrientation->_consigne = 0;
    this->_consigneDist->calculConsigne(_deltaDist);
  }
  
  
//  if (this->_consigneDist->calcEstArrive())
//  {
//    this->_consigneOrientation->_consigne = 0;
//  }
}

void Robot::calculCommande()
{
  if (this->_typeAsserv == Aucun)
  {
   this->_pidDist->changeEtat(false);
   this->_pidOrientation->changeEtat(false);
  }
  else
  {
   this->_pidDist->changeEtat(true);
   this->_pidOrientation->changeEtat(true);
  }
	
    this->_pidDist
      ->calculCommande(
        this->_consigneDist->_consigne, 
        this->_consigneDist->transformeDeltaDistanceEnConsigne(this->_deltaDist)
        );
    this->_pidOrientation
      ->calculCommande(
        this->_consigneOrientation->_consigne, 
        this->_consigneOrientation->transformeDeltaDistanceEnConsigne(this->_deltaOrient * ENTRAXE)
     );
    
    // calcule des consignes
    float commmandeRoueDroite = this->_pidDist->_commande + this->_pidOrientation->_commande;
    float commmandeRoueGauche = this->_pidDist->_commande - this->_pidOrientation->_commande;
	
	
	// determination du sens
    if (commmandeRoueDroite >= 0)
    {
      this->_sensAvantRoueDroite = true;
    }
    else
    {
      this->_sensAvantRoueDroite = false;
    }
	commmandeRoueDroite = fabs(commmandeRoueDroite);
    
    if (commmandeRoueGauche >= 0)
    {
      this->_sensAvantRoueGauche = true;
    }
    else
    {
      this->_sensAvantRoueGauche = false;
    }
	commmandeRoueGauche = fabs(commmandeRoueGauche);
	
	float rapportDroite = 1.0;
	float rapportGauche = 1.0;
	
	if (commmandeRoueDroite > CONSIGNE_MAX || commmandeRoueGauche > CONSIGNE_MAX)
	{
		if (commmandeRoueDroite > commmandeRoueGauche)
		{
			rapportDroite = 1.0; 
			rapportGauche = commmandeRoueDroite != 0 ? commmandeRoueGauche / commmandeRoueDroite : 1;
		}
		else
		{
			rapportDroite = commmandeRoueGauche != 0 ? commmandeRoueDroite / commmandeRoueGauche : 1;
			rapportGauche = 1.0;
		}
	}
	
	this->_commmandeRoueDroite = (int) (rapportDroite * this->filtreCommandeRoue(commmandeRoueDroite));
	this->_commmandeRoueGauche = (int) (rapportGauche * this->filtreCommandeRoue(commmandeRoueGauche));
}

float Robot::filtreCommandeRoue(float value)
{
  if (value > CONSIGNE_MAX)
  {
    value = CONSIGNE_MAX;
  }
  
  float commande = (VALEUR_MAX_PWM * value / CONSIGNE_MAX) * RATIO_PWM + OFFSET;
  
  if (this->_consigneDist->calcEstArrive() == true || commande <= (OFFSET +  OFFSET / 200))
  {
    commande = 0;
  }
  
  return commande > VALEUR_MAX_PWM ? VALEUR_MAX_PWM : commande;
}

void Robot::avanceDe(float avance, bool avecFreinage, float vitessMax) // en mm
{
  this->_consigneDist->setDemande(avance, avecFreinage);
  this->_consigneDist->setVmaxParcourt(vitessMax);
}

void Robot::tourneDe(float rotation, bool avecFreinage, float vitessMax) // en rad
{
  this->_consigneOrientation->setDemande(rotation * (float) ENTRAXE / 2.0, avecFreinage);
  this->_consigneOrientation->setVmaxParcourt(vitessMax);
}

void Robot::vaEnXY(float x, float y, bool estPointArret, float vitessMax)
{
    float dx = x - this->position.x;
    float dy = y - this->position.y;
    float dTheta = atan2(dy,dx) - this->position.theta;
    float dist = sqrt(dx * dx + dy * dy);
    
    switch (this->_typeAsserv)
    {
      case EnArriere:
        dist = -1.0 * dist;
        dTheta -= PI; 
       break;
       
      case Auto:
        if ((dTheta > PI / 2.0 && dTheta < 3.0 * PI / 2.0) || (dTheta < -PI / 2.0 && dTheta > -3.0 * PI / 2.0))
        {
            dTheta -= PI;
            dist = -1.0 * dist;
        }
       break;
      
      default:
       break; 
    }
        
    while ( dTheta > PI)
    {
      dTheta -= 2*PI;
    }
    
    while ( dTheta < -PI)
    {
      dTheta += 2*PI;
    }

    this->avanceDe(dist, estPointArret, vitessMax);
    this->tourneDe(dTheta);
}

void Robot::vaVersPointSuivant()
{  
    this->_typeDeplacement = (Robot::TypeDeplacement) this->pointSuivant.typeDeplacement;
    this->_typeAsserv = (Robot::TypeAsserv) this->pointSuivant.typeAsserv;
      
    this->vaEnXY(this->pointSuivant.x, this->pointSuivant.y, this->pointSuivant.vitessMax * VITESSE_MAX);
}

bool Robot::estArrive()
{
  return this->_consigneDist->estArrive();
}

bool Robot::passageAuPointSuivant()
{
  if (this->estArrive() && !this->queue.isEmpty())
  {
    _tourneFini = false;
    this->pointSuivant = this->queue.pop();
    return true;
  }

  return false;
}

/*
true == avance
false == recule
*/
bool Robot::quelSens()
{
  if (this->_sensAvantRoueGauche == true && this->_sensAvantRoueDroite == true)
  {
    return 1;
  }
  else if (this->_sensAvantRoueGauche == false && this->_sensAvantRoueDroite != false)
  {
    return 0;
  }
  else
  {
    
    return 2;
    /*
    if (this->_sensAvantRoueGauche == true)
    {
      if ( this->_commmandeRoueGauche > this->_commmandeRoueDroite)
      {
        return true;
      }
      else
      {
        return false;
      }   
    }
    else
    {
      if ( this->_commmandeRoueGauche > this->_commmandeRoueDroite)
      {
        return false;
      }
      else
      {
        return true;
      }  */
    }
  }
}

void Robot::attend(unsigned long attente) // tps ne milliseconde
{
  this->tempsAttenteDeplacement = attente;
  this->debutAttenteDeplacement = millis();
}

bool Robot::estEnAttente()
{
  return millis() - this->debutAttenteDeplacement < this->tempsAttenteDeplacement;
}

void Robot::stopAttente()
{
  this->tempsAttenteDeplacement = 0;
}

void Robot::MAJContaineur(bool isGauche, int pos)
{
  int posDepose = !isGauche ? 25 : 140;
  int posAjoutVerre = !isGauche ? 93 : 82;
  int posMaintien = !isGauche ? 97: 78;
  int posfermer = !isGauche ? 160 : 5;
  
  Servo* servo = !isGauche ? this->servoArG : this->servoArD;
  
  switch (pos)
  {
    case 0: //depose
      servo->write(posDepose);
    break;
    case 1: //ajout verre
      servo->write(posAjoutVerre);
    break;
    case 2: // maintien
      servo->write(posMaintien);
    break;
    case 3: //fermer
      servo->write(posfermer);
    break;
  }
  
}


