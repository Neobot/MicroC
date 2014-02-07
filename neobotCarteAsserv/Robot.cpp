#include "Robot.h"

Robot::Robot(Servo* servoArG, Servo* servoArD, float periodAsserv, float x, float y, float theta)
    : _pidDist(ACTIVE_PID_DISTANCE, KP_DISTANCE, KD_DISTANCE),
      _pidOrientation(ACTIVE_PID_ANGLE, KP_ANGLE, KD_ANGLE),
      _consigneDist(VITESSE_MAX, ACCELARATION_MAX_EN_REEL_LIN, periodAsserv, 1.4, 5),
      _consigneOrientation(VITESSE_MAX_ROT, ACCELARATION_MAX_EN_REEL_ROT, periodAsserv, 0.05)
{
    _tourneFini = false;
    _pingReceived = false;

    _typeDeplacement = TournePuisAvance;
    _typeAsserv = Auto;

    Point point;
    point.x = x;
    point.y = y;
    point.theta = theta;

    servoArG = servoArG;
    servoArD = servoArD;

    teleport(point);
    forceObjectif(point);

    _thetaTotal = 0.0;
    _deltaDist = 0.0;
    _deltaOrient = 0.0;
    _periodAsserv = periodAsserv;
    _commmandeRoueGauche = 0;
    _commmandeRoueDroite = 0;

    pasPrecendentGauche = 0.0;
    pasPrecendentDroit = 0.0;

    _sensAvantRoueGauche = true;
    _sensAvantRoueDroite = true;
}

void Robot::teleport(Point point)
{
    position = point;
    flush();
    pointSuivant = position;
    
}

void Robot::forceObjectif(Point point)
{
    pointSuivant = point;
}

void Robot::ajoutPoint(Point point)
{ 
    queue.push(point);
}

void Robot::ajoutPoint(float x, float y, bool pointArret, int typeDeplacement, float vitessMax)
{ 
    Point point;
    point.x = x;
    point.y = y;
    point.pointArret = pointArret;
    point.vitessMax = vitessMax;
    point.typeDeplacement = (Point::TypeDeplacement) typeDeplacement;

    queue.push(point);
}

void Robot::flush()
{
    queue.clear();
}

void Robot::stop()
{
    flush();

    pointSuivant.x = position.x + _consigneDist._distDcc * cos(position.theta);
    pointSuivant.y = position.y + _consigneDist._distDcc * sin(position.theta);
}

void Robot::majPosition(float pasRoueGauche, float pasRoueDroite)
{
    float distGauche = ((float) COEF_CORRECTION_ROUE_FOLLE_GAUCHE) *  (pasRoueGauche - pasPrecendentGauche) * COEFF_CONVERTION_PAS_METRE;
    float distDroite = ((float) COEF_CORRECTION_ROUE_FOLLE_DROITE) *  (pasRoueDroite - pasPrecendentDroit) *  COEFF_CONVERTION_PAS_METRE;
    float k = 1.0;

    _deltaDist = (float)( distGauche + distDroite) / 2.0;
    _deltaOrient = (float) ( distDroite - distGauche ) / (float) ENTRAXE;

    if (_deltaOrient != 0.0)
    {
        k = 2.0 * sin (_deltaOrient / 2.0) / _deltaOrient ; //approx circulaire
    }

    float dX = k * _deltaDist * cos( position.theta + _deltaOrient / 2.0);
    float dY = k * _deltaDist * sin( position.theta + _deltaOrient / 2.0);

    float deriveX = CORFUGE * _deltaOrient * dY;
    float deriveY = - CORFUGE * _deltaOrient * dX;

    position.x += dX + deriveX;
    position.y += dY + deriveY;
    position.theta += _deltaOrient;

    pasPrecendentGauche = pasRoueGauche;
    pasPrecendentDroit = pasRoueDroite;
}


void Robot::calculConsigne()
{
    float thetaDemande = _deltaOrient * ENTRAXE;

    if (_typeDeplacement == TourneEtAvance)
    {
        _consigneDist.calculConsigne(_deltaDist);
        _consigneOrientation.calculConsigne(thetaDemande);
    }
    else if (_typeDeplacement == TournePuisAvance)
    {
        if (_consigneOrientation.calcEstArrive() == false && !_tourneFini)
        {
            _consigneDist._consigne = 0;
            _consigneOrientation.calculConsigne(thetaDemande);
        }
        else
        {
            if (!_tourneFini)
            {
                _tourneFini = true;
            }
            _consigneDist._consigne = _consigneDist.calculConsigne(_deltaDist);
            _consigneOrientation.calculConsigne(thetaDemande);
        }
    }
    else if (_typeDeplacement == TourneSeulement)
    {
        _consigneDist._consigne = 0;
        _consigneOrientation.calculConsigne(thetaDemande);
    }
    else if (_typeDeplacement == AvanceSeulement)
    {
        _consigneOrientation._consigne = 0;
        _consigneDist.calculConsigne(_deltaDist);
    }


    //  if (_consigneDist.calcEstArrive())
    //  {
    //    _consigneOrientation._consigne = 0;
    //  }
}

void Robot::calculCommande()
{
    if (_typeAsserv == Aucun)
    {
        _pidDist.changeEtat(false);
        _pidOrientation.changeEtat(false);
    }
    else
    {
        _pidDist.changeEtat(true);
        _pidOrientation.changeEtat(true);
    }

    _pidDist.calculCommande(
                _consigneDist._consigne,
                _consigneDist.transformeDeltaDistanceEnConsigne(_deltaDist)
                );
    _pidOrientation.calculCommande(
                _consigneOrientation._consigne,
                _consigneOrientation.transformeDeltaDistanceEnConsigne(_deltaOrient * ENTRAXE)
                );
    
    // calcule des consignes
    float commmandeRoueDroite = _pidDist._commande + _pidOrientation._commande;
    float commmandeRoueGauche = _pidDist._commande - _pidOrientation._commande;

    float rapportDroite = 1.0;
    float rapportGauche = 1.0;

    if (fabs(commmandeRoueDroite) > CONSIGNE_MAX || fabs(commmandeRoueGauche) > CONSIGNE_MAX)
    {
        if (fabs(commmandeRoueDroite) > fabs(commmandeRoueGauche))
        {
            rapportDroite = 1.0;
            rapportGauche = commmandeRoueDroite != 0 ? fabs(commmandeRoueGauche) / fabs(commmandeRoueDroite) : 1;
        }
        else
        {
            rapportDroite = commmandeRoueGauche != 0 ? fabs(commmandeRoueDroite) / fabs(commmandeRoueGauche) : 1;
            rapportGauche = 1.0;
        }
    }

    _commmandeRoueDroite = (int) (rapportDroite * filtreCommandeRoue(commmandeRoueDroite));
    _commmandeRoueGauche = (int) (rapportGauche * filtreCommandeRoue(commmandeRoueGauche));
}

float Robot::filtreCommandeRoue(float value)
{
    if (value > CONSIGNE_MAX)
    {
        value = CONSIGNE_MAX;
    }
    else if (value < -CONSIGNE_MAX)
    {
        value = -CONSIGNE_MAX;
    }

    if (_consigneDist.calcEstArrive() == true)
    {
        value = 0;
    }

    float commande = (VALEUR_MAX_PWM * (value + CONSIGNE_MAX) / (2 * CONSIGNE_MAX)) * RATIO_PWM;

    return commande > VALEUR_MAX_PWM ? VALEUR_MAX_PWM : commande;
}

void Robot::avanceDe(float avance, bool avecFreinage, float vitessMax) // en mm
{
    _consigneDist.setDemande(avance, avecFreinage);
    _consigneDist.setVmaxParcourt(vitessMax);
}

void Robot::tourneDe(float rotation, bool avecFreinage, float vitessMax) // en rad
{
    _consigneOrientation.setDemande(rotation * (float) ENTRAXE / 2.0, avecFreinage);
    _consigneOrientation.setVmaxParcourt(vitessMax);
}

void Robot::vaEnXY(float x, float y, bool estPointArret, float vitessMax)
{
    float dx = x - position.x;
    float dy = y - position.y;
    float dTheta = atan2(dy,dx) - position.theta;
    float dist = sqrt(dx * dx + dy * dy);
    
    switch (_typeAsserv)
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

    avanceDe(dist, estPointArret, vitessMax);
    tourneDe(dTheta);
}

void Robot::vaVersPointSuivant()
{  
    _typeDeplacement = (Robot::TypeDeplacement) pointSuivant.typeDeplacement;
    _typeAsserv = (Robot::TypeAsserv) pointSuivant.typeAsserv;

    vaEnXY(pointSuivant.x, pointSuivant.y, pointSuivant.vitessMax * VITESSE_MAX);
}

bool Robot::estArrive()
{
    return _consigneDist.estArrive();
}

bool Robot::passageAuPointSuivant()
{
    if (estArrive() && !queue.isEmpty())
    {
        _tourneFini = false;
        pointSuivant = queue.pop();
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
    if (_sensAvantRoueGauche == true && _sensAvantRoueDroite == true)
    {
        return 1;
    }
    else if (_sensAvantRoueGauche == false && _sensAvantRoueDroite != false)
    {
        return 0;
    }
    else
    {

        return 2;
        /*
    if (_sensAvantRoueGauche == true)
    {
      if ( _commmandeRoueGauche > _commmandeRoueDroite)
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
      if ( _commmandeRoueGauche > _commmandeRoueDroite)
      {
        return false;
      }
      else
      {
        return true;
      }
    }*/
    }
}

void Robot::attend(unsigned long attente) // tps ne milliseconde
{
    tempsAttenteDeplacement = attente;
    debutAttenteDeplacement = millis();
}

bool Robot::estEnAttente()
{
    return millis() - debutAttenteDeplacement < tempsAttenteDeplacement;
}

void Robot::stopAttente()
{
    tempsAttenteDeplacement = 0;
}

void Robot::MAJContaineur(bool isGauche, int pos)
{
    int posDepose = !isGauche ? 25 : 140;
    int posAjoutVerre = !isGauche ? 93 : 82;
    int posMaintien = !isGauche ? 97: 78;
    int posfermer = !isGauche ? 160 : 5;

    Servo* servo = !isGauche ? servoArG : servoArD;

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


