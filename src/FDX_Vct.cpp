/*
 * FDX_Vct.cpp
 *
 * Copyright 2015 Joaqu�n Monteagudo G�mez <kindos7@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */

/*
    C++ library (source file)
    FDX_Vct
    2D vectors
*/

/*
    Version N� 1.0 (dd/mm/yy, 06/01/2015 -> 13/01/2015)
*/

/*
    Preprocessor
*/

/* Includes */

//Header file
#include "../include/FDX_Vct.hpp"

namespace fdx{ namespace arrow
{
    /*
        Functions
    */

    /*Comparison of coordinates and angles*/

    /*Checks if two doubles are almost identical (using EPSILON fast method)
      Not safe if the values are close to zero, use with caution*/
    bool almost_equal(double a, double b)
    {
        return
            (
                (a==b)//First, fast comparision
                ||
                (std::abs(a-b)<EPSILON_COMP)//Second comparision, not safe for all range of values
            );
    }

    /*
        Vector
    */

    /* Vector friends */

    /*Operators and conversors*/

    //Equality operator: same X and Y coordinates
    bool operator==(const Vct &a, const Vct &b)
    {
        return (almost_equal(a.x,b.x)&&almost_equal(a.x,b.x));
    }

    //Inequality operator: opposite of equality
    bool operator!=(const Vct &a, const Vct &b)
    {
        return !(a==b);
    }

    /*Operators and conversors*/

    //Add operator: adds X and Y coordinates by separate to create a new vector
    Vct operator+(const Vct &a, const Vct &b)
    {
        return Vct (a.x+b.x,a.y+b.y);
    }

    //Subtract operator: subtracts the X and Y coordinates by separate to create a new vector (first - second)
    Vct operator-(const Vct &a, const Vct &b)
    {
        return Vct (a.x-b.x,a.y-b.y);
    }

    //Product operator: multiply the X and Y coordinates by a coefficient
    Vct operator* (const Vct &a, Vct::Mod coefficient)
    {
        return Vct (a,coefficient);
    }

    //Product operator: multiply the X and Y coordinates by a coefficient
    Vct operator* (Vct::Mod coefficient, const Vct &a)
    {
        return Vct (a,coefficient);
    }

    //Input operator
    std::istream &operator>> (std::istream &is, Vct &v)
    {
        is >> v.x >> v.y;
        return is;
    }

    //Output operator
    std::ostream &operator<< (std::ostream &os, Vct &v)
    {
        os << "(" << v.x << "," << v.y << ")";
        return os;
    }

    /*
        Vector methods
    */

    /*Module operations*/

    //Change the module of this vector to the given value (direction inverted if the number is negative)
    void Vct::limmod (Mod nm)
    {
        //If this vector is not null, multiply it by a coefficient
        if (operator bool())
        {
            operator*=(nm/mod());
        }
        //If the operator is null...
        else
        {
            /*...if null is allowed, no operation is performed,
              else, we use the angle to turn it into an unary vector*/
            if (!DEF_NULL_ALLOWED)//null vector is not allowed
                set_ang_mod(DEF_NULL_UNI_ANGLE,nm);
        }
    }

    //Turn this vector into the unitary vector
    void Vct::unitary()
    {
        //If this vector is not null, multiply it by the inverse of its module
        if (operator bool())
        {
            operator*=(1/mod());
        }
        //If the operator is null...
        else
        {
            /*...if null is allowed, no operation is performed,
              else, we use the angle to turn it into an unary vector*/
            if (!DEF_NULL_ALLOWED)//null vector is not allowed
                set_ang_mod(DEF_NULL_UNI_ANGLE);
        }
    }

    /*Angle operations*/

    /*Define a vector with a given angle and module
      Static version for defining new vectors and non-static for modifying this vector*/

    //Creates a new vector with a given angle and module
    Vct Vct::mk_ang_mod (Mod angle, Mod module)
    {
        /*Return a new vector with the X and Y coordinates calculated
          using the angle and the module, reversed if needed*/
        return Vct   (
                            std::cos(angle)*(DEF_REVER_X?-module:module)//X coordinate
                            ,
                            std::sin(angle)*(DEF_REVER_Y?-module:module)//Y coordinate
                        );
    }

    //Sets this vector with the given angle and module
    void Vct::set_ang_mod (Mod nangle, Mod nmodule)
    {
        //Get X using the cos and invert if needed
        x=std::cos(nangle)*(DEF_REVER_X?-nmodule:nmodule);
        //Get Y using the sin and invert if needed
        y=std::sin(nangle)*(DEF_REVER_Y?-nmodule:nmodule);
    }

    //Sets this vector as an unitary vector with the given angle
    void Vct::set_ang_mod (Mod angle)
    {
        //Get X using the cos and invert if needed
        x=DEF_REVER_X?-std::cos(angle):std::cos(angle);
        //Get Y using the sin and invert if needed
        y=DEF_REVER_Y?-std::sin(angle):std::sin(angle);
    }

    //Get the angle that this vectors forms with +OX or another vector

    //Get the angle that this vectors forms with +OX, the angle it's within [0,2*PI)
    Vct::Mod Vct::angle() const
    {
        //If the vector is not null, we calculate the angle it forms with +OX
        if (operator bool())
        {
            //Copy of this vector because it will be modified
            Vct c(*this);
            //Reverse its coordinates if needed
            c.rev_cord();
            //Calculate the angle using atan2
            Mod rv=std::atan2(c.y,c.x);
            //If the angle is negative, we turn it into positive
            if (rv<0)
                return (2*PI)+rv;
            //Else, we just return the angle without changing it
            return rv;
        }
        //If the vector is null, the specified constant is returned
        return DEF_NULL_UNI_ANGLE;
    }

}}//End of namespace
