#include "Robot.h"
#include "IOConfig.h"
#include "Logger.h"

Robot::Robot(Adafruit_TCS34725 *colorSensor1, Adafruit_TCS34725 *colorSensor2, float periodAsserv, float x, float y, float theta) :
	_pidDist(ACTIVE_PID_DISTANCE, KP_DISTANCE, KD_DISTANCE, KI_DISTANCE),
	_pidOrientation(ACTIVE_PID_ANGLE, KP_ANGLE, KD_ANGLE, KI_ANGLE),
	_consigneDist(VITESSE_MAX, ACCELARATION_MAX_EN_REEL_LIN, periodAsserv, COEFF_FREINAGE_DIST, DIST_ARRIVE_DIST),
	_consigneOrientation(VITESSE_MAX_ROT, ACCELARATION_MAX_EN_REEL_ROT, periodAsserv, COEFF_FREINAGE_ANG, DIST_ARRIVE_ANG), _logger(0)
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
	_commandeRoueGauche = MAX_PWM_MOTORS / 2.0;
	_commandeRoueDroite = MAX_PWM_MOTORS / 2.0;

    pasPrecendentGauche = 0.0;
    pasPrecendentDroit = 0.0;

	_colorSensor[ColorSensor1] = colorSensor1;
	_colorSensor[ColorSensor2] = colorSensor2;
	
	_colorSensorEnabled[ColorSensor1] = false;
	_colorSensorEnabled[ColorSensor2] = false;
}

void Robot::setLogger(Logger *logger)
{
	_logger = logger;
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

bool Robot::checkBlocked(int speedL, int speedMotL, int speedR, int speedMotR)
{
	// speed are in mm/s
	bool blocked = false;

	if (speedL > 50 && speedMotL < 20)
		blocked = true;

	if (speedR > 50 && speedMotR < 20)
		blocked = true;

	return blocked;
}


void Robot::calculConsigne()
{
    float thetaDemande = _deltaOrientRad * ENTRAXE_MM / 2.0;

#ifdef DEBUG_PID
	_logger->print("Delta dist: ");
	_logger->print(_deltaDistMm);
	_logger->print(", Theta demande: ");
	_logger->println(thetaDemande);
	_logger->print("vitesse courante: ");
	_logger->print(_consigneDist._vitessCourrante);
	_logger->print(" acc max parcours: ");
	_logger->println(_consigneDist._accelerationMaxParcourt);
#endif

    if (_typeDeplacement == TourneEtAvance)
    {
        _consigneDist.calculConsigne(_deltaDistMm);
        _consigneOrientation.calculConsigne(thetaDemande);
    }
    else if (_typeDeplacement == TournePuisAvance)
    {
        if (_consigneOrientation.calcEstArrive() == false && !_tourneFini)
        {
            _consigneDist._consigne = 0.0;
            _consigneOrientation.calculConsigne(thetaDemande);
        }
        else
        {
            if (!_tourneFini)
            {
                _tourneFini = true;
            }
            _consigneDist.calculConsigne(_deltaDistMm);
            _consigneOrientation._consigne = 0.0;
        }
    }
    else if (_typeDeplacement == TourneSeulement)
    {
        _consigneDist._consigne = 0.0;
        _consigneOrientation.calculConsigne(thetaDemande);
    }
    else if (_typeDeplacement == AvanceSeulement)
    {
        _consigneOrientation._consigne = 0.0;
        _consigneDist.calculConsigne(_deltaDistMm);
    }

}

void Robot::calculCommande()
{
	_pidDist.calculCommande(
                _consigneDist._consigne,
                _consigneDist.transformeDeltaDistanceEnConsigne(_deltaDistMm)
                );
    _pidOrientation.calculCommande(
                _consigneOrientation._consigne,
                _consigneOrientation.transformeDeltaDistanceEnConsigne(_deltaOrientRad * ENTRAXE_MM / 2.0)
                );

    // calcule des commandes moteurs
    // -1 devant... ils doivent √™tre cabl√© √† l'envers...
    _commandeRoueDroite = (int) filtreCommandeRoue(-1.0*(_pidDist._commande - _pidOrientation._commande));
    _commandeRoueGauche = (int) filtreCommandeRoue(-1.0*(_pidDist._commande + _pidOrientation._commande));
	//_logger->print("*** "); _logger->println(_consigneDist.transformeDeltaDistanceEnConsigne(_deltaDistMm));
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

    /*if (_consigneDist.calcEstArrive() == true)
    {
        value = 0;
    }*/

	return (MAX_PWM_MOTORS * (value + COMMANDE_MOTEUR_MAX) / (2 * COMMANDE_MOTEUR_MAX));
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

#ifdef DEBUG_PID
	_logger->print("dTheta: ");
	_logger->print(dTheta);
	_logger->print(" dist: ");
	_logger->println(dist);
#endif
    
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

	vaEnXY(pointSuivant.x, pointSuivant.y, pointSuivant.pointArret, pointSuivant.vitessMax * VITESSE_MAX / 100);
}

bool Robot::estArrive()
{
    return _consigneDist.estArrive();
}

bool Robot::passageAuPointSuivant()
{
	if (estArrive())
    {
        _tourneFini = false;
		if (!queue.isEmpty())
		{
			pointSuivant = queue.pop();

			if (pointSuivant.typeAsserv == Aucun)
			{
				_pidDist.changeEtat(false);
				_pidOrientation.changeEtat(false);
			}
			else
			{
				_pidDist.changeEtat(true);
				_pidOrientation.changeEtat(true);
			}
		}
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
	if (sensorId >= 0 && sensorId < ColorSensorCount && _colorSensorEnabled[sensorId] == false)
	{
		_colorSensorEnabled[sensorId] = true;
		_colorSensorStatus[sensorId] = ColorUnknown;
		_colorSensor[sensorId]->setInterrupt(false); //allume la led
	}
}

void Robot::disableColorSensor(int sensorId)
{
	if (sensorId >= 0 && sensorId < ColorSensorCount && _colorSensorEnabled[sensorId] == true)
	{
		_colorSensorEnabled[sensorId] = false;
		_colorSensorStatus[sensorId] = ColorUnknown;
		_colorSensor[sensorId]->setInterrupt(true); //eteint la led
	}
}

bool Robot::isColorSensorEnabled(int sensorId)
{
	return sensorId >= 0 && sensorId < ColorSensorCount && _colorSensorEnabled[sensorId];
}

bool Robot::colorSensorValueHasChanged(int sensorId, ColorSensorState *color)
{
	if (isColorSensorEnabled(sensorId))
	{
		float h, s, l;

		_colorSensor[sensorId]->getColorInHSL(&h, &s, &l); // l = 0 -> white, l = 1 -> black

		if ((h >= 330 || h <= 30) && s > 0.5 && l > 0.3 && l < 0.8)			// red: hue = 0¬∞
			*color = ColorRed;
		else if (h >= 30 && h <= 90 && s > 0.5 && l > 0.3 && l < 0.8)	// yellow: hue = 60¬∞
			*color = ColorYellow;
		else
			*color = ColorBlack;

		if (*color != _colorSensorStatus[sensorId])		// color has changed
		{
			_colorSensorStatus[sensorId] = *color;
			return true;
		}
	}

	return false;
}

void Robot::startPump(int pumpId)
{
	switch (pumpId)
	{
	case 0:
		digitalWrite(PIN_MOTEUR_1, HIGH);
		digitalWrite(PIN_MOTEUR_2, HIGH);
		break;
	case 1:
		digitalWrite(PIN_MOTEUR_1, HIGH);
		break;
	case 2:
		digitalWrite(PIN_MOTEUR_2, HIGH);
		break;
	default:
		break;
	}
}

void Robot::stopPump(int pumpId)
{
	switch (pumpId)
	{
	case 0:
		digitalWrite(PIN_MOTEUR_1, LOW);
		digitalWrite(PIN_MOTEUR_2, LOW);
		break;
	case 1:
		digitalWrite(PIN_MOTEUR_1, LOW);
		break;
	case 2:
		digitalWrite(PIN_MOTEUR_2, LOW);
		break;
	default:
		break;
	}
}

