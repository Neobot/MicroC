#ifndef PID_H
#define PID_H

//nombre d'echantillons pour filtrer le pid :)
#define NB_VALUE_FOR_PID_INTEGRAL 10
#define VALUE_MAX_PID 10000.0;

class PID
{
    public:
    
    enum EtatPID{
        Actif = 1, 
        Desactive = 0,
    };
		
    float _kp; // gain proportionnel
    float _kd; // gain derivé
    float _ki; //gain integrale
    
    float _correction; // la correction a ajouter
    
    float _erreur; // erreur courante
    float _lastErreur; // precedente erreur
    float _prevErreurs[NB_VALUE_FOR_PID_DERIVATE]; // tableau des erreurs précedentes pour le gain integrale
    int _index; // index courant du tableau

    EtatPID _etatCourant; // etat du PID

    
    PID(bool actif, float kp = 1.0, float kd = 0.0, float ki = 0.0);
		
    float calculCorrection(float error);
    void changeEtat(bool actif);
    void reset();
    void addPrevErreur();
        
};

#endif // PID_H

