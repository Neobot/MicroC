#ifndef PID_H
#define PID_H

//nombre d'echantillons pour filtrer le pid :)
#define NB_VALUE_FOR_PID_INTEGRAL 10

// Pour sature le PID au cas ou ;)
#define VALUE_MAX_PID 30000.0;

class PID
{
public:

    enum EtatPID{
        Actif = 1, 
        Desactive = 0,
    };

    float _kp; // gain proportionnel
    float _kd; // gain derivÔøΩ
    float _ki; //gain integrale
    
    float _correction; // la correction a ajouter
    float _precedenteConsigne;
	float _precedentePrecedenteConsigne;
    float _commande;
    
    float _erreur; // erreur courante
    float _lastErreur; // precedente erreur
    float _prevErreurs[NB_VALUE_FOR_PID_INTEGRAL]; // tableau des erreurs prÔøΩcedentes pour le gain integrale
    int _index; // index courant du tableau
    
    float _valMaxCorrection;

    EtatPID _etatCourant;

    PID(bool actif, float kp = 0.0, float kd = 0.0, float ki = 0.0);

    float calculCommande(float consigne, float distanceRealise);
    void seuilPid();
    void changeEtat(bool actif);
    void reset();
    void addPrevErreur();

};

#endif // PID_H


