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

#include "Arduino.h"
#include "Pid.h"

PID::PID(bool actif, float kp, float kd)
{
	this->changeEtat(actif);
	this->reset();
	
	this->_kp = kp;
	this->_kd = kd;
}

float PID::calculCommande(float consigne, float distanceRealiseEnNormeConsigne)
{
	this->_commande = consigne;
	
	if (this->_etatCourant == Actif)
	{
		this->_erreur = this->_precedenteConsigne - distanceRealiseEnNormeConsigne;
		this->_commande += this->_kp * this->_erreur - this->_kd * distanceRealiseEnNormeConsigne;
        }
	this->_precedenteConsigne = consigne;
	
	return this->_commande;
}

void PID::reset()
{
	this->_consigne = 0.0;
	this->_precedenteConsigne = 0.0;
	this->_erreur = 0.0;
	this->_commande = 0.0;
}

void PID::changeEtat(bool actif)
{
	if (actif)
		this->_etatCourant = Actif;
	else
		this->_etatCourant = Desactive;
}

