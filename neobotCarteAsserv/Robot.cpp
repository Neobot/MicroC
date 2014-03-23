#include "Robot.h"

Robot::Robot(Adafruit_TCS34725 *colorSensor1, Adafruit_TCS34725 *colorSensor2, float periodAsserv, float x, float y, float theta) :
	_pidDist(ACTIVE_PID_DISTANCE, KP_DISTANCE, KD_DISTANCE),
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

    teleport(point);
    forceObjectif(point);

    _thetaTotal = 0.0;
    _deltaDistMm = 0.0;
    _deltaOrientRad = 0.0;
    _periodAsserv = periodAsserv;
    _commandeRoueGauche = 0;
    _commandeRoueDroite = 0;

    pasPrecendentGauche = 0.0;
    pasPrecendentDroit = 0.0;

	_colorSensor[0] = colorSensor1;
	_colorSensor[1] = colorSensor2;
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
    float distGaucheMm = ((float) COEF_CORRECTION_ROUE_FOLLE_GAUCHE) *  (pasRoueGauche - pasPrecendentGauche) * COEFF_CONVERTION_PAS_MM;
    float distDroiteMm = ((float) COEF_CORRECTION_ROUE_FOLLE_DROITE) *  (pasRoueDroite - pasPrecendentDroit) *  COEFF_CONVERTION_PAS_MM;
    float k = 1.0;

    _deltaDistMm = (float)( distGaucheMm + distDroiteMm) / 2.0;
    _deltaOrientRad = (float) ( distDroiteMm - distGaucheMm ) / (float) ENTRAXE_MM;

    if (_deltaOrientRad != 0.0)
    {
        k = 2.0 * sin (_deltaOrientRad / 2.0) / _deltaOrientRad ; //approx circulaire
    }

    float dX = k * _deltaDistMm * cos( position.theta + _deltaOrientRad / 2.0);
    float dY = k * _deltaDistMm * sin( position.theta + _deltaOrientRad / 2.0);

    float deriveX = CORFUGE * _deltaOrientRad * dY;
    float deriveY = - CORFUGE * _deltaOrientRad * dX;

    position.x += dX + deriveX;
    position.y += dY + deriveY;
    position.theta += _deltaOrientRad;

    pasPrecendentGauche = pasRoueGauche;
    pasPrecendentDroit = pasRoueDroite;
}


void Robot::calculConsigne()
{
    float thetaDemande = _deltaOrientRad * ENTRAXE_MM;

    if (_typeDeplacement == TourneEtAvance)
    {
        _consigneDist.calculConsigne(_deltaDistMm);
        _consigneOrientation.calculConsigne(thetaDemande);
    }
    else if (_typeDeplacement == TournePuisAvance)
    {
        if (_consigneOrientation.calcEstArrive() == false && !_tourneFini)
        {
            _consigneDist.calculConsigne(_deltaDistMm);
            _consigneOrientation.calculConsigne(thetaDemande);
        }
        else if (_typeDeplacement == TournePuisAvance)
        {
            if (!_tourneFini)
            {
                _tourneFini = true;
            }
            _consigneDist._consigne = _consigneDist.calculConsigne(_deltaDistMm);
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
        _consigneDist.calculConsigne(_deltaDistMm);
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
                _consigneDist.transformeDeltaDistanceEnConsigne(_deltaDistMm)
                );
    _pidOrientation.calculCommande(
                _consigneOrientation._consigne,
                _consigneOrientation.transformeDeltaDistanceEnConsigne(_deltaOrientRad * ENTRAXE_MM)
                );
    
    // calcule des commandes moteurs
    float commandeRoueDroite = _pidDist._commande + _pidOrientation._commande;
    float commandeRoueGauche = _pidDist._commande - _pidOrientation._commande;

    float rapportDroite = 1.0;
    float rapportGauche = 1.0;

    if (fabs(commandeRoueDroite) > COMMANDE_MOTEUR_MAX || fabs(commandeRoueGauche) > COMMANDE_MOTEUR_MAX)
    {
        if (fabs(commandeRoueDroite) > fabs(commandeRoueGauche))
        {
            rapportDroite = 1.0;
            rapportGauche = commandeRoueDroite != 0 ? fabs(commandeRoueGauche) / fabs(commandeRoueDroite) : 1;
        }
        else
        {
            rapportDroite = commandeRoueGauche != 0 ? fabs(commandeRoueDroite) / fabs(commandeRoueGauche) : 1;
            rapportGauche = 1.0;
        }
    }

    _commandeRoueDroite = (int) (rapportDroite * filtreCommandeRoue(commandeRoueDroite));
    _commandeRoueGauche = (int) (rapportGauche * filtreCommandeRoue(commandeRoueGauche));
}

float Robot::filtreCommandeRoue(float value)
{
    if (value > COMMANDE_MOTEUR_MAX)
    {
        value = COMMANDE_MOTEUR_MAX;
    }
    else if (value < -COMMANDE_MOTEUR_MAX)
    {
        value = -COMMANDE_MOTEUR_MAX;
    }

    if (_consigneDist.calcEstArrive() == true)
    {
        value = 0;
    }

	float commande = (MAX_PWM_MOTORS * (value + COMMANDE_MOTEUR_MAX) / (2 * COMMANDE_MOTEUR_MAX)) * RATIO_PWM;

	return constrain(commande, 0, MAX_PWM_MOTORS);
}

void Robot::avanceDe(float avance, bool avecFreinage, float vitessMax) // en mm
{
    _consigneDist.setDemande(avance, avecFreinage);
    _consigneDist.setVmaxParcourt(vitessMax);
}

void Robot::tourneDe(float rotation, bool avecFreinage, float vitessMax) // en rad
{
    _consigneOrientation.setDemande(rotation * (float) ENTRAXE_MM / 2.0, avecFreinage);
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

//true == avance
//false == recule
bool Robot::quelSens()
{
    if (_commandeRoueGauche + _commandeRoueDroite > 0)
        return 1;
    else
        return 0;
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

void Robot::enableColorSensor(int sensorId)
{
	_colorSensorEnabled[sensorId] = 1;
	_colorSensor[sensorId]->setInterrupt(false);
}


void Robot::disableColorSensor(int sensorId)
{
	_colorSensorEnabled[sensorId] = 0;

	_colorSensor[sensorId]->setInterrupt(true);
}

bool Robot::isColorSensorEnabled(int sensorId)
{
	return _colorSensorEnabled[sensorId];
}

void Robot::readColorSensor(int sensorId)
{
	if (_colorSensor[sensorId]->isEnabled())
	{
		float h, s, l;

		_colorSensor[sensorId]->getColorInHSL(&h, &s, &l);

		// to do: convert to meaninfull color (red, yellow, nothing)
	}
}
