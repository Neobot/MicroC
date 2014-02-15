#ifndef PID_H
#define PID_H

class PID
{
public:

    float _kp;
    float _kd;
    float _commande;

    PID(bool actif, float kp = 1.0, float kd = 1.0);

    float calculCommande(float consigne, float distanceRealise);
    void changeEtat(bool actif);
    void reset();

    enum EtatPID{Actif, Desactive};

    float _consigne;
    float _precedenteConsigne;
    float _erreur;

    EtatPID _etatCourant;

};

#endif // PID_H

