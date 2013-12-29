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

#ifndef TRAPEZE_H
#define TRAPEZE_H

class Trapeze
{
  public:
    
    float _derive;
    float _prevVal;

    float _limitDevPos;
    
    float _limitAcePos;
    float _limitAceNeg;
    
    void setLimitVit(float vit);
    void setLimitAcc(float accPos, float accNeg);
    float filter(float distRest, float vMaxSuivante = 0.0);
}

#endif