/*
 * FDX_Geo.hpp
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
    FDX_Geo
    Physic implementation of basic shapes
*/

/*
    Version 0.1 (dd/mm/yy, 29/06/2015 -> )
*/

/*
    Preprocessor
*/

/*Header guard*/
#ifndef _FDX_GEO_H_
#define _FDX_GEO_H_


/* Includes */

//2D vectors
#include "../FDX_Vct/FDX_Vct.hpp"

/* Defines */

/*Constants*/

/*Macros*/

namespace fdx { namespace arrow
{
    /*
        Class declarations
    */

    class Shp;//Generic shape

    class Crl;//Circle shape

    class Pnt;//Point shape

    class Rct;//Rectangle shape

    class Set;//Set of real values with two limits

    /*
        Function prototypes
    */

    /* Shapes */

    /*Contact*/

    //Contact between two shapes (Crl, Pnt)
    bool contact_crl_pnt (const Shp& s1, const Shp& s2);

    /*TTH*/

    //Time from the first shape to hit the second at the given speed

    //(Crl, Pnt)
    Vct::Mod tth_crl_pnt (const Shp& s1, const Shp& s2, const Vct& speed);

    /*Move against a shape*/

    //Move the first shape against the other at the given speed

    //(Crl, Pnt)
    Vct mov_against_crl_pnt (const Shp& s1, const Shp& s2, const Vct& speed);

    /*
        Data types
     */

    /* Typedefs */

    /* Classes */

    //Generic shape
    class Shp
    {
        /* Constructors, copy control */

        /*Constructors*/

        protected:

            //Default constructor
            Shp() = default;

            //Default copy constructor
            Shp (const Shp &ns) = default;

        /*Copy control*/

        protected:

            //Default copy constructor
            Shp& operator= (const Shp &ns) = default;

        public:

            //Virtual destructor (allows the class to be extended)
            virtual ~Shp() {}

        /* Position */

        /*Get*/

        public:

            //Get the center of the shape
            virtual Vct get_pos_center () const = 0;

            //Get the upper left corner of the rectangle that contains the shape completly
            virtual Vct get_pos_corner () const = 0;

        /*Set*/

        public:

            //Set the center of the shape
            virtual void set_pos_center (const Vct &ncenter) = 0;

            //Set the upper left corner of the rectangle that contains the shape completly
            virtual void set_pos_corner (const Vct &ncorner) = 0;

        /* Size */

        /*Get*/

        public:

            //Get the size of the circle that contains the shape completly
            virtual Vct::Mod get_size () const = 0;

            //Get the size (diagonal) of the rectangle that contains the shape completly
            virtual Vct get_diagonal () const = 0;

        /*Set*/

        public:

            //Set the size of the circle that contains the shape completly
            virtual void set_size (Vct::Mod nsize) = 0;

            //Set the size (diagonal) of the rectangle that contains the shape completly
            virtual void set_diagonal (const Vct &ndiag) = 0;

        /* Move */

        public:

            //Move the shape by the given vector
            virtual void mov (const Vct &m) = 0;

        /* Contact */

        public:

            //Contact with a generic shape
            virtual bool contact (const Shp &s) const = 0;

            //Contact with a circle
            virtual bool contact (const Crl &c) const = 0;

            //Contact with a point
            virtual bool contact (const Pnt &p) const = 0;

            //Contact with a rectangle
            virtual bool contact (const Rct &r) const = 0;

        /* Time to hit */

        public:

            //TTH a generic shape at a given speed
            virtual Vct::Mod tth (const Shp &s, const Vct &speed) const = 0;

            //TTH a circle at a given speed
            virtual Vct::Mod tth (const Crl &c, const Vct &speed) const = 0;

            //TTH a point at a given speed
            virtual Vct::Mod tth (const Pnt &p, const Vct &speed) const = 0;

            //TTH a rectangle at a given speed
            virtual Vct::Mod tth (const Rct &r, const Vct &speed) const = 0;

        /* Movement against a shape */

        public:

            //Movement against a generic shape at a given speed
            virtual Vct mov_against (const Shp &s, const Vct &speed) const = 0;

            //Movement against a circle at a given speed
            virtual Vct mov_against (const Crl &c, const Vct &speed) const = 0;

            //Movement against a point at a given speed
            virtual Vct mov_against (const Pnt &p, const Vct &speed) const = 0;

            //Movement against a rectangle at a given speed
            virtual Vct mov_against (const Rct &r, const Vct &speed) const = 0;
    };

    //Circle
    class Crl : public Shp
    {
        /* Attributes */

        /*Position*/

        private:

            Vct r;//Center of the circle

        /*Size*/

        private:

            Vct::Mod s;//Radius of the circle

        /* Constructors, copy control */

        /*Constructors*/

        public:

            //Default constructor
            Crl() = default;

            //Complete constructor
            Crl (const Vct &nr, Vct::Mod ns)
            :r(nr), s(ns)
            {}

            //Default copy constructor
            Crl (const Crl &nc) = default;

        /*Copy control*/

        public:

            //Default copy operator
            Crl& operator= (const Crl &nc) = default;

            //Destructor
            virtual ~Crl() {}

        /* Position */

        /*Get*/

        public:

            //Get the center of the shape
            Vct get_pos_center () const
            {
                return r;
            }

            //Get the upper left corner of the rectangle that contains the shape completly
            Vct get_pos_corner () const
            {
                return r+Vct(-s,-s);
            }

        /*Set*/

        public:

            //Set the center of the shape
            void set_pos_center (const Vct &ncenter)
            {
                r=ncenter;
            }

            //Set the upper left corner of the rectangle that contains the shape completly
            void set_pos_corner (const Vct &ncorner)
            {
                r=ncorner+Vct(s,s);
            }

        /* Size */

        /*Get*/

        public:

            //Get the size of the circle that contains the shape completly
            Vct::Mod get_size () const
            {
                return s;
            }

            //Get the size (diagonal) of the rectangle that contains the shape completly
            Vct get_diagonal () const
            {
                return Vct(2*s,2*s);
            }

        /*Set*/

        public:

            //Set the size of the circle that contains the shape completly
            void set_size (Vct::Mod nsize)
            {
                s=nsize;
            }

            //Set the size (diagonal) of the rectangle that contains the shape completly
            void set_diagonal (const Vct &ndiag)
            {
                s=std::min(ndiag.x,ndiag.y);
            }

        /* Move */

        public:

            //Move the shape by the given vector
            void mov (const Vct &m)
            {
                r+=m;
            }

        /* Contact */

        public:

            //Contact with a generic shape
            bool contact (const Shp &s) const;

            //Contact with a circle
            bool contact (const Crl &c) const;

            //Contact with a point
            bool contact (const Pnt &p) const;

            //Contact with a rectangle
            bool contact (const Rct &r) const;

        /* Time to hit */

        public:

            //TTH a generic shape at a given speed
            Vct::Mod tth (const Shp &s, const Vct &speed) const;

            //TTH a circle at a given speed
            Vct::Mod tth (const Crl &c, const Vct &speed) const;

            //TTH a point at a given speed
            Vct::Mod tth (const Pnt &p, const Vct &speed) const;

            //TTH a rectangle at a given speed
            Vct::Mod tth (const Rct &r, const Vct &speed) const;

        /* Movement against a shape */

        public:

            //Movement against a generic shape at a given speed
            Vct mov_against (const Shp &s, const Vct &speed) const;

            //Movement against a circle at a given speed
            Vct mov_against (const Crl &c, const Vct &speed) const;

            //Movement against a point at a given speed
            Vct mov_against (const Pnt &p, const Vct &speed) const;

            //Movement against a rectangle at a given speed
            Vct mov_against (const Rct &r, const Vct &speed) const;
    };

    //Point
    class Pnt : public Shp
    {
        /* Attributes */

        /*Position*/

        private:

            Vct r;//Point position

        /* Constructors, copy control */

        /*Constructors*/

        public:

            //Default constructor
            Pnt() = default;

            //Complete constructor
            Pnt (const Vct &nr)
            :r(nr)
            {}

            //Default copy constructor
            Pnt (const Pnt &np) = default;

        /*Copy control*/

        public:

            //Default copy operator
            Pnt& operator= (const Pnt &np) = default;

            //Destructor
            virtual ~Pnt() {}

        /* Position */

        /*Get*/

        public:

            //Get the center of the shape
            Vct get_pos_center () const
            {
                return r;
            }

            //Get the upper left corner of the rectangle that contains the shape completly
            Vct get_pos_corner () const
            {
                return r;
            }

        /*Set*/

        public:

            //Set the center of the shape
            void set_pos_center (const Vct &ncenter)
            {
                r=ncenter;
            }

            //Set the upper left corner of the rectangle that contains the shape completly
            void set_pos_corner (const Vct &ncorner)
            {
                r=ncorner;
            }

        /* Size */

        /*Get*/

        public:

            //Get the size of the circle that contains the shape completly
            Vct::Mod get_size () const
            {
                return 0;
            }

            //Get the size (diagonal) of the rectangle that contains the shape completly
            Vct get_diagonal () const
            {
                return Vct(0,0);
            }

        /*Set*/

        public:

            //Set the size of the circle that contains the shape completly
            void set_size (Vct::Mod)
            {
                return;//This function does nothing on a point
            }

            //Set the size (diagonal) of the rectangle that contains the shape completly
            void set_diagonal (const Vct &)
            {
                return;//This function does nothing on a point
            }

        /* Move */

        public:

            //Move the shape by the given vector
            void mov (const Vct &m)
            {
                r+=m;
            }

        /* Contact */

        public:

            //Contact with a generic shape
            bool contact (const Shp &s) const;

            //Contact with a circle
            bool contact (const Crl &c) const;

            //Contact with a point
            bool contact (const Pnt &p) const;

        /* Time to hit */

        public:

            //TTH a generic shape at a given speed
            Vct::Mod tth (const Shp &s, const Vct &speed) const;

            //TTH a circle at a given speed
            Vct::Mod tth (const Crl &c, const Vct &speed) const;

            //TTH a point at a given speed
            Vct::Mod tth (const Pnt &p, const Vct &speed) const;

        /* Movement against a shape */

        public:

            //Movement against a generic shape at a given speed
            Vct mov_against (const Shp &s, const Vct &speed) const;

            //Movement against a circle at a given speed
            Vct mov_against (const Crl &c, const Vct &speed) const;

            //Movement against a point at a given speed
            Vct mov_against (const Pnt &p, const Vct &speed) const;
    };

    //Rectangle
    class Rct : public Shp
    {
        /* Attributes */

        /*Position*/

        private:

            Vct r;//Upper left corner position

            Vct s;//Size, from the upper left corner to the opposite corner

        /* Constructors, copy control */

        /*Constructors*/

        public:

            //Default constructor
            Rct() = default;

            //Complete constructor
            Rct (const Vct &nr, const Vct &ns)
            :r(nr), s(ns)
            {}

            //Default copy constructor
            Rct (const Rct &nr) = default;

        /*Copy control*/

        public:

            //Default copy constructor
            Rct& operator= (const Rct &nr) = default;

            //Destructor
            virtual ~Rct() {}

        /* Position */

        /*Get*/

        public:

            //Get the center of the shape
            Vct get_pos_center () const
            {
                return r+0.5*s;
            }

            //Get the upper left corner of the rectangle that contains the shape completly
            Vct get_pos_corner () const
            {
                return r;
            }

        /*Set*/

        public:

            //Set the center of the shape
            void set_pos_center (const Vct &ncenter)
            {
                r=ncenter-0.5*s;
            }

            //Set the upper left corner of the rectangle that contains the shape completly
            void set_pos_corner (const Vct &ncorner)
            {
                r=ncorner;
            }

        /* Size */

        /*Get*/

        public:

            //Get the size of the circle that contains the shape completly
            Vct::Mod get_size () const
            {
                return (0.5*s).mod();
            }

            //Get the size (diagonal) of the rectangle that contains the shape completly
            Vct get_diagonal () const
            {
                return s;
            }

        /*Set*/

        public:

            //Set the size of the circle that contains the shape completly
            void set_size (Vct::Mod nsize)
            {
                s=Vct(2*nsize,2*nsize);
            }

            //Set the size (diagonal) of the rectangle that contains the shape completly
            void set_diagonal (const Vct &ndiag)
            {
                s=ndiag;
            }

        /* Move */

        public:

            //Move the shape by the given vector
            void mov (const Vct &m)
            {
                r+=m;
            }

        /* Contact */

        public:

            //Contact with a generic shape
            bool contact (const Shp &s) const;

            //Contact with a circle
            bool contact (const Crl &c) const;

            //Contact with a point
            bool contact (const Pnt &p) const;

        /* Time to hit */

        public:

            //TTH a generic shape at a given speed
            Vct::Mod tth (const Shp &s, const Vct &speed) const;

            //TTH a circle at a given speed
            Vct::Mod tth (const Crl &c, const Vct &speed) const;

            //TTH a point at a given speed
            Vct::Mod tth (const Pnt &p, const Vct &speed) const;

        /* Movement against a shape */

        public:

            //Movement against a generic shape at a given speed
            Vct mov_against (const Shp &s, const Vct &speed) const;

            //Movement against a circle at a given speed
            Vct mov_against (const Crl &c, const Vct &speed) const;

            //Movement against a point at a given speed
            Vct mov_against (const Pnt &p, const Vct &speed) const;
    };

    //Set of real values with two limits
    class Set
    {
        /* Configuration */

        /*Data types*/
        public:

            //Type of limits
            typedef double Limit;

            //Type of values
            typedef double Value;

        /*Constants*/
        private:

            static constexpr Limit DEF_MIN_LIMIT=0,DEF_MAX_LIMIT=0;//Default set
            static constexpr Limit DEF_MIN_NULL=0,DEF_MAX_NULL=-1;//Default null set (holds no values)

        /* Attributes */

        /*Limits*/
        private:

            Limit min_limit, max_limit;

        /* Limit members */

        /*Validity*/
        public:

            //Check for validity of the set
            bool valid()
            {
                return min_limit<=max_limit;
            }

            //Swap the limit
            void swap_limits()
            {
                Limit temp=min_limit;
                min_limit=max_limit;
                max_limit=temp;
            }

        /*Set/get*/
        public:

            //Set limit

            void set_min(Limit il)
            {
                min_limit=il;
            }

            void set_max(Limit il)
            {
                max_limit=il;
            }

            //Safe set (swaps limits if necessary)

            void set_safe_min(Limit il)
            {
                set_min(il);
                if (!valid())
                    swap_limits();
            }

            void set_safe_max(Limit il)
            {
                set_max(il);
                if (!valid())
                    swap_limits();
            }

            //Get limits

            Limit get_min()
            {
                return min_limit;
            }

            Limit get_max()
            {
                return max_limit;
            }

        /*Values*/
        public:

            //Check if a value is in the set
            bool check_value(Value v)
            {
                return(min_limit<=v&&v<=max_limit)
            }

        /*Size*/
        public:

            //Get the size
            Limit get_size()
            {
                return max_limit-min_limit;
            }

            //Is elemental
            bool elemental()
            {
                return min_limit==max_limit;
            }

            //Get the center
            Limit get_middle()
            {
                return (min_limit+max_limit)/2.0;
            }

        /* Constructors, copy control */

        /*Constructors*/
        public:

            //Complete constructor
            Set(Limit imin, Limit imax)
            :min_limit(imin),max_limit(imax)
            {}

            //Default constructor
            Set()
            :Set(DEF_MIN_LIMIT,DEF_MAX_LIMIT)
            {}

            //Copy constructor
            Set(const Set& s)
            :Set(s.min_limit,s.max_limit)
            {}

        /*Operators*/
        public:

            //Assignment operator
            Set& operator= (const Set& s)
            {
                min_limit=s.min_limit;
                max_limit=s.max_limit;
                return *this;
            }

            //Mulitply and assign operator
            void operator*= (Limit m)
            {
                min_limit*=m;
                max_limit*=m;
                if (m<0)
                    swap_limits();
            }

        /* Set methods */

        /*Intersect*/
        public:

            //Return the intersection of two sets
            static Set min_intersect(const Set& s1, const Set& s2)
            {
                return Set(std::max(s1.get_min(),s2.get_min()),std::min(s1.get_max(),s2.get_max()));
            }

            //Returns the big union of two sets (no gaps)
            static Set max_union(const Set &s1, const Set& s2)
            {
                return Set(std::min(s1.get_min(),s2.get_min()),std::max(s1.get_max(),s2.get_max()));
            }

        /*TTH*/
        public:

            //TTH a point
            Set tth(Value v, Value speed)
            {
                Set rv(v-get_min(),v-get_max());//Distance
                rv*=(1.0/speed);//Get the distance to time dividing it by the speed
                return rv;
            }

            //TTH a set
            Set tth(const Set& s, Value speed)
            {
                return max_union(tth(s.get_min(),speed),tth(s.get_max()),speed);
            }

    };

}}//End of namespace

//End of library
#endif // _FDX_GEO_H_
