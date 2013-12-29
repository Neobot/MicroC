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

#ifndef CERCLE_H
#define CERCLE_H

/*
 * Equation du type (x - xc)² + (y - yc)² = r²;
 * de base on fixe c à -1 => ax - y + b =0 => y = ax + b
 */

class Cercle
{
  public:
    
    float _xc;
    float _yc;
    float _r;

}

#endif