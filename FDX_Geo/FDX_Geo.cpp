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

    //Contact between two rects
    bool contact_rct_rct (const Rct &r1, const Rct &r2)
    {
        //Distance between the centers
        Vct rdist(r1.get_pos_center()-r2.get_pos_center());

        //Size of the two rectangles combined
        Vct rsz(r1.get_diagonal()+r2.get_diagonal());
        rsz*=(0.5f);//Get the half size, not the full size

        //Check for contact (size is less than the distance)
        return ((std::abs(rdist.x)<=rsz.x)&&(std::abs(rdist.y)<=rsz.y));
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
    //0=center, 1=non center; sign swaps for other side
    void rel_pos_crlpnt_rct (const Shp& s, const Rct& r, int &px, int &py)
    {
        //X
        if (s.get_pos_center().x<r.get_pos_center().x)
            px=-1;//Left
        else
        {
            if (s.get_pos_center().x<=(r.get_pos_center().x+r.get_diagonal().x))
                px=0;//Center
            else
                px=1;//Right
        }

        //Y
        if (s.get_pos_center().y<r.get_pos_center().y)
            py=-1;//Up
        else
        {
            if (s.get_pos_center().y<=(r.get_pos_center().y+r.get_diagonal().y))
                py=0;//Center
            else
                py=1;//Down
        }
    }

    //Time to hit of a rectangle to a circle at the given speed
    Vct::Mod tth_crlpnt_rct (const Shp& s, const Rct& r, const Vct& speed)
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
                tth=ccopy.tth(Pnt(corner),speed);

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
                    Vct::Coord rct_side=r.get_pos_corner().x,crl_side=ccopy.get_pos_center().x;
                    if (px>0)
                    {
                        rct_side+=r.get_diagonal().x;
                        crl_side-=ccopy.get_size();
                    }
                    else
                    {
                        crl_side+=ccopy.get_size();
                    }

                    tth=tth_coordinate(crl_side,rct_side,speed.x);
                    tte=speed.y<0?tth_coordinate(ccopy.get_pos_center().y,r.get_pos_corner().y,speed.y):tth_coordinate(ccopy.get_pos_center().y,r.get_pos_corner().y+r.get_diagonal().y,speed.y);
                }

                //Up or down
                else
                {
                    //Get the correct coordinates
                    Vct::Coord rct_side=r.get_pos_corner().y,crl_side=ccopy.get_pos_center().y;
                    if (py>0)
                    {
                        rct_side+=r.get_diagonal().y;
                        crl_side-=ccopy.get_size();
                    }
                    else
                    {
                        crl_side+=ccopy.get_size();
                    }

                    tth=tth_coordinate(crl_side,rct_side,speed.y);
                    tte=speed.x<0?tth_coordinate(ccopy.get_pos_center().x,r.get_pos_corner().x,speed.x):tth_coordinate(ccopy.get_pos_center().x,r.get_pos_corner().x+r.get_diagonal().x,speed.x);
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

    //(Rct, Rct)
    //Relative position of the first rectangle to the second
    //0=center, 1=inside contact, 2=border contact, 3=no contact; sign swaps for other side
    void rel_pos_rct_rct (const Rct& r1, const Rct& r2, int &px, int &py)
    {
        //Get the data to process
        Vct d(r2.get_pos_center()-r1.get_pos_center());//Distance between centers
        Vct s((r1.get_diagonal()+r2.get_diagonal()),0.5);//Size of the rectangles combined

        //Process each side separetly

        //X

        //Check for centers aligned
        if (d.x)//Non center
        {
            if (arrow::almost_equal(std::abs(d.x),s.x))//Border contact
            {
                px=2;
            }
            else//Inside or outside
            {
                //Check the type of contact
                if (std::abs(d.x)<s.x)//Inside
                    px=1;
                else//Outside
                    px=3;
            }
        }
        else//Center
            px=0;

        //Adjust the sign
        if (d.x<0)px=-px;

        //Y

        //Check for centers aligned
        if (d.y)//Non center
        {
            if (arrow::almost_equal(std::abs(d.y),s.y))//Border contact
            {
                py=2;
            }
            else//Inside or outside
            {
                //Check the type of contact
                if (std::abs(d.y)<s.y)//Inside
                    py=1;
                else//Outside
                    py=3;
            }
        }
        else//Center
            py=0;

        //Adjust the sign
        if (d.y<0)py=-py;
    }

    //Time from the first rectangle to hit the second
    Vct::Mod tth_rct_rct (const Rct &r1, const Rct &r2, const Vct& speed)
    {
        //Sides

        //R1
        Set r1x(r1.get_pos_corner().x,r1.get_pos_corner().x+r1.get_diagonal().x);//X
        Set r1y(r1.get_pos_corner().y,r1.get_pos_corner().y+r1.get_diagonal().y);//Y

        //R2
        Set r2x(r2.get_pos_corner().x,r2.get_pos_corner().x+r2.get_diagonal().x);//X
        Set r2y(r2.get_pos_corner().y,r2.get_pos_corner().y+r2.get_diagonal().y);//Y

        //Time of contact
        Set ttc(Set::min_intersect(r1x.tth(r2x,speed.x),r1y.tth(r2y,speed.y)));//Intersection between X and Y TTCs

        //Check for contact at the begining
        if (ttc.check_value(0))//If 0 is on the set, the contact starts at the begining
            return 0;
        else//If 0 is not on the set, return the tth (start of ttc) if it's at the left, or 1 if it's at the right
            return ttc.get_min()<0?1:std::min(1.0,ttc.get_min());
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
        Crl ccopy(s.get_pos_center()+speed_free,s.get_size());//Copy of the circle

        int px,py;//Relative positions of the circle
        rel_pos_crlpnt_rct(ccopy,r,px,py);

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
            speed_left=ccopy.mov_against(Pnt(corner),speed_left);
        }
        else//Side contact or center contact
        {
            if (!(px||py))//Center of circle inside rectangle
            {
                speed_left=ccopy.mov_against(Crl(r.get_pos_center(),r.get_size()),speed_left);//Move against the rectangle's center
            }
            else//Side contact
            {
                if (px)//X side
                {
                    if (px<0)//Circle at the left
                    {
                        if (speed_left.x>0)speed_left.x=0;//Restrict movement to the right
                    }
                    else//Circle at the right
                    {
                        if (speed_left.x<0)speed_left.x=0;//Restrict movement to the left
                    }
                }
                else//Y side
                {
                    if (py<0)//Circle at the top
                    {
                        if (speed_left.y>0)speed_left.y=0;//Restrict movement to bottom
                    }
                    else//Circle at the bottom
                    {
                        if (speed_left.y<0)speed_left.y=0;//Restrict movement to the top
                    }
                }
            }
        }

        //Return the not limited movement and the limited movement
        return speed_free+speed_left;
    }


    //(Rct, Rct)
    Vct mov_against_rct_rct (const Rct& r1, const Rct& r2, const Vct& speed)
    {
        //Get the tth from the first rectangle to the second
        Vct::Mod tth=r1.tth(r2,speed);

        //Check if the TTH limits the movement
        if (tth>=1||tth<0)//No limit
            return speed;

        //Speed is limited, process it
        Vct speed_free(speed,tth);//Speed not limited by the tth
        Rct rcopy(r1);//Copy of the rectangle
        rcopy.mov(speed_free);

        //Get the relative position
        int px,py;
        rel_pos_rct_rct(rcopy,r2,px,py);

        //Process the remaining speed
        Vct speed_left(speed-speed_free);

        //Check the type of contact

        //Corner contact
        if (std::abs(px)>=2&&std::abs(py)>=2)
        {
            //Check if the speed goes on the same direction as the corner (inside the Rct)
            if (speed_left.x*px>0&&speed_left.y*py>0)//Only one component of the speed is to be kept (priority on X)
            {
                if (std::abs(speed_left.x)>=std::abs(speed_left.y))
                    speed_left.y=0;//X is kept
                else
                    speed_left.x=0;//Y is kept
            }
            //Else, the speed does not need to be modified
        }
        else//Side contact or center contact
        {
            if (std::abs(px)>=2)//Border contact on left/right
            {
                if (speed_left.x*px>0)//If the speed goes towards the center, limit it
                    speed_left.x=0;
            }
            else if (std::abs(py)>=2)//Border contact on top bottom
            {
                if (speed_left.y*py>0)//If the speed goes towards the center, limit it
                    speed_left.y=0;
            }
            else//Inside contact
            {
                if (speed_left.x*px>0)//X limit
                    speed_left.x=0;
                if (speed_left.y*py>0)//Y limit
                    speed_left.y=0;
            }
        }

        //Return the speed
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
    }

    /*Time to hit*/

    //TTH a generic shape at a given speed
    Vct::Mod Crl::tth (const Shp &s, const Vct &speed) const
    {
        return s.tth(*this,-speed);
    }

    //TTH a circle at a given speed
    Vct::Mod Crl::tth (const Crl &c, const Vct &speed) const
    {
        return arrow::tth_crlpnt_crlpnt(*this,c,speed);
    }

    //TTH a point at a given speed
    Vct::Mod Crl::tth (const Pnt &p, const Vct &speed) const
    {
        return arrow::tth_crlpnt_crlpnt(*this,p,speed);
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
        return arrow::mov_against_crlpnt_crlpnt(*this,c,speed);
    }

    //Movement against a point at a given speed
    Vct Crl::mov_against (const Pnt &p, const Vct &speed) const
    {
        return arrow::mov_against_crlpnt_crlpnt(*this,p,speed);
    }

    //Movement against a rectangle at a given speed
    Vct Crl::mov_against (const Rct &r, const Vct &speed) const
    {
        return arrow::mov_against_crlpnt_rct(*this,r,speed);
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
        return arrow::contact_crlpnt_crlpnt(*this,c);
    }

    //Contact with a point
    bool Pnt::contact (const Pnt &p) const
    {
        return arrow::contact_crlpnt_crlpnt(*this,p);
    }

    //Contact with a rectangle
    bool Pnt::contact (const Rct &r) const
    {
        return arrow::contact_crlpnt_rct(*this,r);
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
        return arrow::tth_crlpnt_crlpnt(*this,c,speed);
    }

    //TTH a point at a given speed
    Vct::Mod Pnt::tth (const Pnt &p, const Vct &speed) const
    {
        return arrow::tth_crlpnt_crlpnt(*this,p,speed);
    }

    //TTH a rectangle at a given speed
    Vct::Mod Pnt::tth (const Rct &r, const Vct &speed) const
    {
        return arrow::tth_crlpnt_rct(*this,r,speed);
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
        return arrow::mov_against_crlpnt_crlpnt(*this,c,speed);
    }

    //Movement against a point at a given speed
    Vct Pnt::mov_against (const Pnt &p, const Vct &speed) const
    {
        return arrow::mov_against_crlpnt_crlpnt(*this,p,speed);
    }

    //Movement against a rectangle at a given speed
    Vct Pnt::mov_against (const Rct &r, const Vct &speed) const
    {
        return arrow::mov_against_crlpnt_rct(*this,r,speed);
    }

    /* Rct */

    /*Contact*/

    //Contact with a generic shape
    bool Rct::contact (const Shp &s) const
    {
        return s.contact(*this);
    }

    //Contact with a circle
    bool Rct::contact (const Crl &c) const
    {
        return arrow::contact_crlpnt_rct(c,*this);
    }

    //Contact with a point
    bool Rct::contact (const Pnt &p) const
    {
        return arrow::contact_crlpnt_rct(p,*this);
    }

    //Contact with a rectangle
    bool Rct::contact (const Rct &r) const
    {
        return arrow::contact_rct_rct(*this,r);
    }

    /*Time to hit*/

    //TTH a generic shape at a given speed
    Vct::Mod Rct::tth (const Shp &s, const Vct &speed) const
    {
        return s.tth(*this,-speed);
    }

    //TTH a circle at a given speed
    Vct::Mod Rct::tth (const Crl &c, const Vct &speed) const
    {
        return arrow::tth_crlpnt_rct(c,*this,-speed);
    }

    //TTH a point at a given speed
    Vct::Mod Rct::tth (const Pnt &p, const Vct &speed) const
    {
        return arrow::tth_crlpnt_rct(p,*this,-speed);
    }

    //TTH a rectangle at a given speed
    Vct::Mod Rct::tth (const Rct &r, const Vct &speed) const
    {
        return arrow::tth_rct_rct(*this,r,speed);
    }

    /*Movement against a shape*/

    //Movement against a generic shape at a given speed
    Vct Rct::mov_against (const Shp &s, const Vct &speed) const
    {
        return -s.mov_against(*this,-speed);
    }

    //Movement against a circle at a given speed
    Vct Rct::mov_against (const Crl &c, const Vct &speed) const
    {
        return -arrow::mov_against_crlpnt_rct(c,*this,-speed);
    }

    //Movement against a point at a given speed
    Vct Rct::mov_against (const Pnt &p, const Vct &speed) const
    {
        return -arrow::mov_against_crlpnt_rct(p,*this,-speed);
    }

    //Movement against a rectangle at a given speed
    Vct Rct::mov_against (const Rct &r, const Vct &speed) const
    {
        return arrow::mov_against_rct_rct(*this,r,speed);
    }

}}//End of namespace
