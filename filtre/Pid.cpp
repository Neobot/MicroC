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

PID::PID(bool actif, float kp, float kd, float ki)
{
	this->changeEtat(actif);
	this->reset();
	
	this->_kp = kp;
	this->_kd = kd;
    this->_ki = ki;
}

void PID::reset()
{
    this->_erreur = 0.0;
    this->_correction = 0.0;
    this->_lastErreur = 0.0;
    this->_index = 0;
  
    for(int i = 0; i < NB_VALUE_FOR_PID_INTEGRAL; ++i)
        this->_prevErreurs[i] = 0;
}

float PID::calculCorrection(float erreur)
{
	this->_correction = 0.0;
  
	if (this->_etatCourant == Actif)
	{
        float derive;
        float integral;
      
        this->_lastErreur = this->_erreur;
        this->_erreur = erreur;
        
        derive = this->_erreur - this->_lastErreur;
        
        integral = 0.0;
        for(int i = 0; i < NB_VALUE_FOR_PID_INTEGRAL; ++i)
          integral += this->_prevErreurs[i];
        
        
        // on sature la correction PID
        this->_correction = min(
          this->_kp * this->_erreur + this->_kd * derive + this->_ki * integral, 
          VALUE_MAX_PID
        );  
        
        this->addPrevErreur();
    }
	
	return this->_correction;
}

void PID::changeEtat(bool actif)
{
	if (actif)
		this->_etatCourant = Actif;
	else
		this->_etatCourant = Desactive;
}

void PID::addPrevErreur()
{
    ++this->_index;
    if (this->_index >= NB_VALUE_FOR_PID_INTEGRAL)
      this->_index =0;
      
    this->_prevErreurs[this->_index] = this->_erreur;
}

