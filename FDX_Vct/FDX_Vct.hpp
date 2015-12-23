/*
 * FDX_Vct.hpp
 *
 * Copyright 2015 Joaquín Monteagudo Gómez <kindos7@gmail.com>
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
    C++ library (header file)
    FDX_Vct
    2D vectors
*/

/*
    Version Nº 1.0 (dd/mm/yy, 06/01/2015 -> 13/01/2015)
*/

/*
    Preprocessor
*/

/*Header guard*/
#ifndef _FDX_VCT_H_
#define _FDX_VCT_H_


/* Includes */

//Standard maths
#include <cmath>

//Input/output streams
#include <iostream>

/* Defines */

/*Constants*/

//Epsilon to compare doubles, not safe when comparing values close to zero
constexpr double EPSILON_COMP = 0.000001;

//PI
constexpr double PI = 3.14159265358979323846;

/*Macros*/

namespace fdx { namespace arrow
{
    /*
        Class definitions
    */
    class Vct;

    /*
        Function prototypes
    */

    /* Comparison of coordinates and angles */

    /*Checks if two doubles are almost identical (using EPSILON fast method)
      Not safe if the values are close to zero, use with caution*/
    bool almost_equal (double a, double b);

    /* Vct friends */

    /*Operators and conversors*/

    //Equality operator: same X and Y coordinates
    bool operator== (const Vct &a, const Vct &b);

    //Inequality operator: opposite of equality
    bool operator!= (const Vct &a, const Vct &b);

    //Add operator: adds X and Y coordinates by separate to create a new vector
    Vct operator+ (const Vct &a, const Vct &b);

    //Subtract operator: subtracts the X and Y coordinates by separate to create a new vector (first - second)
    Vct operator- (const Vct &a, const Vct &b);

    //Product operator: multiply the X and Y coordinates by a coefficient
    Vct operator* (const Vct &a, double coefficient);

    //Product operator: multiply the X and Y coordinates by a coefficient
    Vct operator* (double coefficient, const Vct &a);

    //Input operator
    std::istream &operator>> (std::istream &is, Vct &v);

    //Output operator
    std::ostream &operator<< (std::ostream &os, Vct &v);

    /*
        Data types
     */

    /* Typedefs */

    /* Classes */

    //Class that holds a 2D vector in the XY plane.
    class Vct
    {
        /* Types and constants */

        /*Types used in the class*/

        public:

            //Type of the coordinates X and Y
            typedef double Coord;
            //Type of the module of a vector, angles and coefficients
            typedef double Mod;

        /*Constants*/

        private:

            //Default values for X and Y
            static constexpr Coord DEFX=0.0, DEFY=0.0;

            /*The null vector (0,0) is allowed (true) or not (false)
              If it's allowed, it will remain (0,0) when it's changed
              If it's not allowed, a defined angle will be used to make it not null when needed*/
            static constexpr bool DEF_NULL_ALLOWED=false;

            //Angle that forms the unitary version of (0,0) with +OX in radians (regular math angle)
            static constexpr Mod DEF_NULL_UNI_ANGLE=0.0;

            /*Reverse the axes X and Y
              If true, a vector will switch the sign of it's coordinate
              ONLY when performing angle related operations. That includes
              setting the angle of a vector (vector is modified) and getting
              the angle of a vector (vector is not modified.*/
            static constexpr bool DEF_REVER_X=false, DEF_REVER_Y=true;

        /* Attributes */

        /*Coordinates*/

        public:

            Coord x,y;//Coordinates X and Y

        /* Copy control, constructors */

        public:

            //Default constructor
            Vct()
            : x(DEFX), y(DEFY)
            {}

            //Coordinate constructor
            Vct (Coord nx, Coord ny)
            : x(nx), y(ny)
            {}

            //Copy constructor (same as default)
            Vct (const Vct &v)
            : Vct (v.x,v.y)
            {}

            //Copy constructor with coefficient
            Vct (const Vct &v, Mod coefficient)
            : x(v.x*coefficient), y(v.y*coefficient)
            {}

            //Destructor (virtual for derivated classes) (same as default)
            virtual ~Vct() = default;

            //Copy assignment operator
            Vct& operator= (const Vct &v)
            {
                x=v.x;
                y=v.y;
                return *this;
            }

        /* Operators and conversors */

        public:

            //Equality operator: same X and Y coordinates
            friend bool operator== (const Vct &a, const Vct &b);

            //Inequality operator
            friend bool operator!= (const Vct &a, const Vct &b);

            //Add operator: adds X and Y coordinates by separate to create a new vector
            friend Vct operator+ (const Vct &a, const Vct &b);

            //Subtract operator: subtracts the X and Y coordinates by separate to create a new vector (first - second)
            friend Vct operator- (const Vct &a, const Vct &b);

            //Product operator: multiply the X and Y coordinates by a coefficient
            friend Vct operator* (const Vct &a, Mod coefficient);

            //Product operator: multiply the X and Y coordinates by a coefficient
            friend Vct operator* (Mod coefficient, const Vct &a);

            //Input operator
            friend std::istream &operator>> (std::istream &is, Vct &v);

            //Output operator
            friend std::ostream &operator<< (std::ostream &os, Vct &v);

            //Bool converter: X and Y coordinates are not 0
            operator bool() const
            {
                return x||y;
            }

            //Negate method: changes the coordinates of this vector to their contrary sign value
            void inv_dir()
            {
                x=-x;
                y=-y;
            }

            //Negate operator: creates a new vector with the opposite coordinates of this vector
            Vct operator-() const
            {
                return Vct (-x,-y);
            }

            //Add to operator: adds one vector to this vector
            void operator+= (const Vct &v)
            {
                x+=v.x;
                y+=v.y;
            }

            //Substract from operator: substract one vector to this vector
            void operator-= (const Vct &v)
            {
                x-=v.x;
                y-=v.y;
            }

            //Multiply by operator: muliplies this vector by a coefficient
            void operator*= (Mod coefficient)
            {
                x*=coefficient;
                y*=coefficient;
            }

        /* Acces methods */

        public:

            //Set the X and Y coordinates
            void setXY (Coord nx, Coord ny)
            {
                x=nx;
                y=ny;
            }

        /* Reverse coordinate control */
        private:

            //Reverse the coordinates if needed
            void rev_cord()
            {
                if (DEF_REVER_X)
                    x=-x;
                if (DEF_REVER_Y)
                    y=-y;
            }

        /* Module operations */
        public:

            //Get the squared module of this vector (efficient checking)
            Mod sq_mod() const
            {
                return x*x+y*y;
            }

            //Get the module of this vector
            Mod mod() const
            {
                return std::sqrt(sq_mod());
            }

            //Change the module of this vector to the given value (direction inverted if the number is negative)
            void limmod (Mod nm);

            //Turn this vector into the unitary vector
            void unitary();

        /* Angle operations */

        /*Define a vector with a given angle and module
          Static version for defining new vectors and non-static for modifying this vector*/

        public:

            //Creates a new vector with a given angle and module
            static Vct mk_ang_mod (Mod angle, Mod module);

            //Sets this vector with the given angle and module
            void set_ang_mod (Mod nangle, Mod nmodule);

        private:

            //Sets this vector as an unitary vector with the given angle
            void set_ang_mod (Mod angle);

        /*Get the angle that this vectors forms with +OX or another vector*/

        public:

            //Get the angle that this vectors forms with +OX, the angle it's within [0,2*PI)
            Mod angle() const;

            /*Get the angle that the guide vector forms with this vector
              Represents the angle that the guide vectors turns to reach the this vector
              If the angle is positive, this turn is counter-clockwise
              If it's negative, it's clockwise
              The angle is within (-2*PI,2*PI)*/
            Mod angle (const Vct &v) const
            {
                return angle()-v.angle();
            }

            //Get the angle that the guide vector forms with this vector, but the guide vector is given by its angle
            Mod angle (Mod nangle) const
            {
                return angle()-nangle;
            }

        /* Rotation of vectors */

        public:

            //Rotate this vector, adding the given angle to its angle
            void rot (Mod nangle)
            {
                set_ang_mod(nangle+angle(),mod());
            }

        /* Check of directions */

        /*Check if two vectors have the same direction*/

        public:

            //Check if the direction between this vector and the given vector is the saame
            bool same_dir (const Vct &v) const
            {
                return almost_equal(angle(),v.angle());
            }

            //Fast check to see if this and another vectors that are linearly dependant have the same direction
            bool same_dir_fast (const Vct &v) const
            {
                //Check that the coordinates of this vector and the other have the same sign
                //Try to check if they are different, and if they are not, they must be the same

                return  !
                        (
                            (x*v.x<0)//Check X
                            ||
                            (y*v.y<0)//Check Y
                        );
            }

        /* Descomposition of vectors using a guide vector */

        /*Tangencial part of this vector*/

        public:

            //Return a new vector that is the tangencial part of this vector to a given guide vector

            //Guide vector given as a vector
            Vct tan_part (const Vct &v) const
            {
                return mk_ang_mod(v.angle(),mod_tan_part(v));
            }

            //Guide vector given by angle
            Vct tan_part (Mod nangle) const
            {
                return mk_ang_mod (nangle,mod_tan_part(nangle));
            }

            //Module of the tangencial part of this vector to a given guide

            //Guide vector given as a vector
            Mod mod_tan_part(const Vct &v) const
            {
                return mod()*std::cos(angle(v));
            }

            //Guide vector given as an angle
            Mod mod_tan_part(Mod nangle) const
            {
                return mod()*std::cos(angle(nangle));
            }

        /*Normal part of this vector*/

        public:

            //Return a new vector that is the normal part of this vector to a given guide vector

            //Guide vector given as a vector
            Vct nor_part (const Vct &v) const
            {
                return mk_ang_mod(v.angle()+PI/2,mod_nor_part(v));
            }

            //Guide vector given by angle
            Vct nor_part (Mod nangle) const
            {
                return mk_ang_mod(nangle+PI*2,mod_nor_part(nangle));
            }

            //Module of the normal part of this vector to a given guide

            //Guide vector given as a vector
            Mod mod_nor_part(const Vct &v) const
            {
                return mod()*std::sin(angle(v));
            }

            //Guide vector given as an angle
            Mod mod_nor_part(Mod nangle) const
            {
                return mod()*std::sin(angle(nangle));
            }
    };

}}//End of namespace

//End of library
#endif
