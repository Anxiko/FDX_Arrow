/*
 * FDX_Geo.cpp
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
    C++ library (source file)
    FDX_Geo
    Physic implementation of basic shapes
*/

/*
    Version 0.1 (dd/mm/yy, 29/06/2015 -> )
*/

/*
    Preprocessor
*/

/* Includes */

//Header file
#include "FDX_Geo.hpp"

namespace fdx{ namespace arrow
{
    /*
        Functions
    */

    /* Shapes */

    /*Contact*/

    //Contact between two shapes (Crl/Pnt, Crl/Pnt)
    bool contact_crlpnt_crlpnt (const Shp& s1, const Shp& s2)
    {
        Vct::Mod dist=s1.get_size()+s2.get_size();//Size of the shapes (radius for Crl, 0 for Pnt)
        return ((s1.get_pos_center()-s2.get_pos_center()).sq_mod())<(dist*dist);//If the distance between centers is less than the size, the shapes are in contact
    }

    //Get the minimum absolute value of a set
    Vct::Mod min_abs_set (Vct::Mod min_set, Vct::Mod max_set)
    {
        //If 0 is on the set, it will always be the minimum
        if (min_set<=0&&0<=max_set)//0 is on the set, return it
            return 0;
        else//0 is not in the set, the min abs value is one of the extremes
            return min_set>0?min_set:-max_set;
    }

    //Get the minimum distance between rectangle and a circle/point
    Vct mindist_crlpnt_rct(const Shp &s, const Rct &r)
    {
        //Get the distance between the shapes
        Vct d(r.get_pos_corner()-s.get_pos_center());

        //Get the minium distance between vector between the shapes
        return Vct(arrow::min_abs_set(d.x,d.x+r.get_diagonal().x),arrow::min_abs_set(d.y,d.y+r.get_diagonal().y));
    }

    //Contact between a rectangle and a circle
    bool contact_crlpnt_rct (const Shp &s, const Rct &r)
    {
        return mindist_crlpnt_rct(s,r).sq_mod()<=s.get_size()*s.get_size();
    }

    /*TTH*/

    //Time from the first shape to hit the second at the given speed

    //(Crl/Pnt, Crl/Pnt)
    Vct::Mod tth_crlpnt_crlpnt (const Shp& s1, const Shp& s2, const Vct& speed)
    {
        //If they are alredy in contact
        if (s1.contact(s2))
            return 0;//TTH is zero

        else//They are not in contact
        {
            //If the speed is not null
            if (speed)//They can collide
            {
                //Get the distance
                Vct d(s1.get_pos_center()-s2.get_pos_center());

                //Get the component of the distance tangencial to the speed
                Vct::Mod dx(d.mod_tan_part(speed));

                //Get the sum of the circles size
                Vct::Mod sz=s1.get_size()+s2.get_size();

                //Get the a, b and c coefficientes of the second degree ecuation
                double
                    ac=speed.sq_mod(),
                    bc=2*dx*speed.mod(),
                    cc=d.sq_mod()-sz*sz;

                    //Get the value of the discriminant
                    double disc=bc*bc-4*ac*cc;

                    //Check that the 2nd degree equation has solution
                    //If the equation has no solution
                    if (disc<0)//Then there is no contact
                        return -1;
                    else//Has solution
                    {
                        //Find the two solutions
                        double
                                sol1=(-bc-std::sqrt(disc))/(2*ac),
                                sol2=(-bc+std::sqrt(disc))/(2*ac);

                        //Find the final solution
                        double sol=std::min(sol1,sol2);//Final solution is the lesser of the two
                        if (sol<0) return -1;//If the solution is negative then there is no contact
                        return fdx::arrow::almost_equal(0,sol)?0:sol;//If it is positive, return the value (approximate it to zero)
                    }
            }
            else//The speed is null, they won't collide
                return -1;
        }
    }

    //(Crl/Pnt, Rct)

    //Time for first coordinate to hit the second at the given speed
    Vct::Mod tth_coordinate(Vct::Coord c1, Vct::Coord c2, Vct::Coord v)
    {
        //Check if there is speed
        if (v)
            return (c2-c1)/v;
        else//No speed, will always or never hit
        {
            if (almost_equal(c1,c2))//Always hitting
                return 0;
            else//Never hitting
                return -1;
        }
    }

    //Relative position of this circle/point to the rectangle
    void rel_pos_crlpnt_rct (const Crl& c, const Rct& r, int &px, int &py)
    {
        //X
        if (c.get_pos_center().x<r.get_pos_center().x)
            px=-1;//Left
        else
        {
            if (c.get_pos_center().x<=(r.get_pos_center().x+r.get_diagonal().x))
                px=0;//Center
            else
                px=1;//Right
        }

        //Y
        if (c.get_pos_center().y<r.get_pos_center().y)
            py=-1;//Up
        else
        {
            if (c.get_pos_center().y<=(r.get_pos_center().y+r.get_diagonal().y))
                py=0;//Center
            else
                py=1;//Down
        }
    }

    //Time to hit of a rectangle to a circle at the given speed
    Vct::Mod tth_crlpnt_rct (const Crl& s, const Rct& r, const Vct& speed)
    {
        //If they are alredy in contact, the TTH is 0
        if (s.contact(r))
            return 0;

        //Get the initial position of the Crl using the Rct as reference
        int px,py;//Relative X and Y positions (-1,0,1)

        rel_pos_crlpnt_rct(s,r,px,py);

        //Iterate through the areas to get the next state

        //Time to hit, time to escape the area and current time
        Vct::Mod tth,tte,t=0;

        //Make a copy of the circle
        Crl ccopy(s.get_pos_center(),s.get_size());

        //Loop through the areas
        while(true)
        {
            //Detect the area

            //Corner contact
            if (px&&py)
            {
                //Get the corner of the rect
                Vct corner(r.get_pos_corner());
                if (px>0)corner.x+=r.get_diagonal().x;
                if (py>0)corner.y+=r.get_diagonal().y;

                //Get the tth
                tth=ccopy.mov_against(Pnt(corner),speed);

                //Get the tte
                Vct::Mod ttex,ttey;//Time to scape to x or y
                ttex=tth_coordinate(ccopy.get_pos_center().x,corner.x,speed.x);
                ttey=tth_coordinate(ccopy.get_pos_center().y,corner.y,speed.y);

                if (ttex<0||ttey<0)
                    tte=std::max(ttex,ttey);
                else
                    tte=std::min(ttex,ttey);
            }
            //Side contact
            else
            {
                //Left or right
                if (px)
                {
                    //Get the correct coordinates
                    Vct::Coord r=r.get_pos_corner().x,p=r.get_pos_center().x;
                    if (px>0)
                    {
                        r+=r.get_diagonal().x;
                        p-=ccopy.get_size();
                    }
                    else
                    {
                        p+=ccopy.get_size();
                    }

                    tth=tth_coordinate(p,r,speed.x);
                    tte=std::min(tth_coordinate(ccopy.get_pos_center().y,r.get_pos_corner().y,speed.y),tth_coordinate(ccopy.get_pos_center().y,r.get_pos_corner().y+r.get_diagonal().y,speed.y));
                }

                //Up or down
                else
                {
                    //Get the correct coordinates
                    Vct::Coord r=r.get_pos_corner().y,p=r.get_pos_center().y;
                    if (py>0)
                    {
                        r+=r.get_diagonal().y;
                        p-=ccopy.get_size();
                    }
                    else
                    {
                        p+=ccopy.get_size();
                    }

                    tth=tth_coordinate(p,r,speed.y);
                    tte=std::min(tth_coordinate(ccopy.get_pos_center().x,r.get_pos_corner().x,speed.x),tth_coordinate(ccopy.get_pos_center().x,r.get_pos_corner().x+r.get_diagonal().x,speed.x));
                }
            }

            //TTH and TTE calculated, process data

            //No hit, no escape
            if (tth<0&&tte<0)
                return -1;

            //Escape
            if (tth<0||(tte<tth&&tte>=0))
            {
                //Progress to next update
                t+=tte;
                ccopy.mov(speed*tte);

                if (speed.x>0&&px<1)px++;
                else if (speed.x<0&&px>-1)px--;
                if (speed.y>0&&py<1)py++;
                else if (speed.y<0&&py>-1)py--;
            }

            //Hit
            else
                return tth+t;
        }
    }


    /*Move against a shape*/

    //Move the first shape against the other at the given speed

    //(Crl, Pnt)
    Vct mov_against_crlpnt_crlpnt (const Shp& s1, const Shp& s2, const Vct& speed)
    {
        //Get the time to hit
        Vct::Mod t=tth_crlpnt_crlpnt(s1,s2,speed);

        //Check if the speed needs to be restricted
        //If it will hit in this tick
        if ((t>=0)&&(t<1))//Restrict the speed
        {
            //Get the distance
            Vct d(s2.get_pos_center()-s1.get_pos_center());

            //Check if centers match
            if (d)//Centers don't match
            {
                //Check if the shapes were in contact
                if (t==0)//Shapes are in contact, restrict the tangencial part of the speed if needed
                {
                    //Check if the part of the speed that is tangencial to the distance goes in the same direction
                    //If is the same direction
                    if (speed.mod_tan_part(d)>0)//Restrict the movement
                        return speed.nor_part(d);//The tangencial part is restricted and only the normal part remains
                    else//If the direction is the opposite, the speed remains untouched
                        return speed;
                }
                else//Shapes are not in contact, restrict movement using the tth
                {
                    return speed*t;
                }
            }
            else//Centers match
                return speed;//Free movement
        }
        else//It won't hit on this tick, don't restrict the speed
            return speed;
    }

    //(Crl/Pnt, Rct)
    Vct mov_against_crlpnt_rct (const Shp& s, const Rct& r, const Vct& speed)
    {
        //Get the tth from this circle to the rectangle
        Vct::Mod tth=s.tth(r,speed);

        //Check if the TTH limits the movement
        if (tth>=1||tth<0)//No limit
            return speed;

        //Speed is limited, process it
        Vct speed_free(speed,tth);//Speed not limited by the tth
        Crl ccopy(s.get_pos_center(),s.get_size());//Copy of the circle
        ccopy.mov(rv);//Move the copy to the border

        int px,py;//Relative positions of the circle
        rel_pos_rct_crl(ccopy,r,px,py);

        //Process the remaining speed
        Vct speed_left(speed-speed_free);

        //Check the type of contact
        if (px&&py)//Corner contact
        {
            //Get the corner of the rect
            Vct corner(r.get_pos_corner());
            if (px>0)corner.x+=r.get_diagonal().x;
            if (py>0)corner.y+=r.get_diagonal().y;

            //Move the circle against the corner
            speed_left=c.mov_against(Pnt(corner),speed_left);
        }
        else//Side contact or center contact
        {
            if (!(px||py))//Center of circle inside rectangle
                speed_left=c.mov(Pnt(r.get_pos_center(),speed_left));//Move against the rectangle's center
            else//Side contact
            {
                if (px)//X side
                {
                    if (px<0)//Circle at the left
                        if (speed_left.x>0)speed_left.x=0;//Restrict movement to the right
                    else//Circle at the right
                        if (speed_left.x<0)speed_left.x=0;//Restrict movement to the left
                }
                else//Y side
                {
                    if (py<0)//Circle at the top
                        if (speed_left.y>0)speed_left.y=0;//Restrict movement to bottom
                    else//Circle at the bottom
                        if (speed_left.y<0)speed_left.y=0;//Restrict movement to the top
                }
            }
        }

        //Return the not limited movement and the limited movement
        return speed_free+speed_left;
    }

    /* Crl */

    /*Contact*/

    //Contact with a generic shape
    bool Crl::contact (const Shp &s) const
    {
        return s.contact(*this);
    }

    //Contact with a circle
    bool Crl::contact (const Crl &c) const
    {
        return arrow::contact_crlpnt_crlpnt(*this,c);
    }

    //Contact with a point
    bool Crl::contact (const Pnt &p) const
    {
        return arrow::contact_crlpnt_crlpnt(*this,p);
    }

    //Contact with a rectangle
    bool Crl::contact (const Rct &r) const
    {
        return arrow::contact_crlpnt_rct(*this,r);

    /*Time to hit*/

    //TTH a generic shape at a given speed
    Vct::Mod Crl::tth (const Shp &s, const Vct &speed) const
    {
        return s.tth(*this,-speed);
    }

    //TTH a circle at a given speed
    Vct::Mod Crl::tth (const Crl &c, const Vct &speed) const
    {
        return arrow::tth_crlpnt_crlpnt_pnt(*this,c,speed);
    }

    //TTH a point at a given speed
    Vct::Mod Crl::tth (const Pnt &p, const Vct &speed) const
    {
        return arrow::tth_crlpnt_crlpnt_pnt(*this,p,speed);
    }

    //TTH a rectangle at a given speed
    Vct::Mod Crl::tth (const Rct &r, const Vct &speed) const
    {
        return arrow::tth_crlpnt_rct(*this,r,speed);
    }

    /*Movement against a shape*/

    //Movement against a generic shape at a given speed
    Vct Crl::mov_against (const Shp &s, const Vct &speed) const
    {
        return -s.mov_against(*this,-speed);
    }

    //Movement against a circle at a given speed
    Vct Crl::mov_against (const Crl &c, const Vct &speed) const
    {
        return arrow::mov_against_crlpnt_crlpnt_pnt(*this,c,speed);
    }

    //Movement against a point at a given speed
    Vct Crl::mov_against (const Pnt &p, const Vct &speed) const
    {
        return arrow::mov_against_crlpnt_crlpnt_pnt(*this,p,speed);
    }

    //Movement against a rectangle at a given speed
    Vct Crl::mov_against (const Pnt &r, const Vct &speed) const
    {
        return arrow::mov_against_crlpnt_crlpnt_rct(*this,r,speed);
    }

    /* Pnt */

    /*Contact*/

    //Contact with a generic shape
    bool Pnt::contact (const Shp &s) const
    {
        return s.contact(*this);
    }

    //Contact with a circle
    bool Pnt::contact (const Crl &c) const
    {
        return arrow::contact_crl_pnt(*this,c);
    }

    //Contact with a point
    bool Pnt::contact (const Pnt &p) const
    {
        return arrow::contact_crl_pnt(*this,p);
    }

    /*Time to hit*/

    //TTH a generic shape at a given speed
    Vct::Mod Pnt::tth (const Shp &s, const Vct &speed) const
    {
        return s.tth(*this,-speed);
    }

    //TTH a circle at a given speed
    Vct::Mod Pnt::tth (const Crl &c, const Vct &speed) const
    {
        return arrow::tth_crl_pnt(*this,c,speed);
    }

    //TTH a point at a given speed
    Vct::Mod Pnt::tth (const Pnt &p, const Vct &speed) const
    {
        return arrow::tth_crl_pnt(*this,p,speed);
    }

    /*Movement against a shape*/

    //Movement against a generic shape at a given speed
    Vct Pnt::mov_against (const Shp &s, const Vct &speed) const
    {
        return -s.mov_against(*this,-speed);
    }

    //Movement against a circle at a given speed
    Vct Pnt::mov_against (const Crl &c, const Vct &speed) const
    {
        return arrow::mov_against_crl_pnt(*this,c,speed);
    }

    //Movement against a point at a given speed
    Vct Pnt::mov_against (const Pnt &p, const Vct &speed) const
    {
        return arrow::mov_against_crl_pnt(*this,p,speed);
    }

}}//End of namespace
