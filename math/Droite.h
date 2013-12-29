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

#ifndef DROITE_H
#define DROITE_H

/*
 * Equation du type ax + cy + b = 0;
 * de base on fixe c à -1 => ax - y + b =0 => y = ax + b
 */

class Droite
{
  public:
    
    float _a;
    float _b;
    float _c;

}

#endif