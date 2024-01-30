/*
   OpticalFlow.cpp Functions to calculate optical flow and odometry

   Copyright (c) 2012 Centeye, Inc. 
   All rights reserved.

   Redistribution and use in source and binary forms, with or without 
   modification, are permitted provided that the following conditions are met:

   Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.

   Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation 
   and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY CENTEYE, INC. ``AS IS'' AND ANY EXPRESS OR 
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
   EVENT SHALL CENTEYE, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
   OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   The views and conclusions contained in the software and documentation are 
   those of the authors and should not be interpreted as representing official 
   policies, either expressed or implied, of Centeye, Inc.
 */

#pragma once

#include <stdio.h>
#include <stdint.h>

class OpticalFlow {

    public:

        /**
         *	Changes current optical flow value by low-pass filter with new.
         *
         *  @param filtered_OF filtered_OF filtered optical-flow value
         *  @param new_OF new filtered_OF filtered optical-flow value
         *  @alpha filter parameter (between 0 and 1)
         */
        static void ofoLPF(int16_t *filtered_OF, int16_t *new_OF, float alpha);

        /**
         *	Adds new optical flow value to accumulation sum iff only new value
         *  falls outside threshold.
         *  @param new_OF new optical-flow value
         *  @param acc_OF pointer to accumulated value
         *  @param threshold threshold value
         *  @return true if value was added, false otherwise
         */
        bool ofoAccumulate(int16_t new_OF, int16_t *acc_OF, int16_t threshold);

        /**
         *	Runs a one-dimensional version of the image interpolation algorithm
         *	(IIA) described in
         *
         *  \@article{Srinivasan1994,<br>
         *  author="Srinivasan, M. V.",<br>
         *  title="An image-interpolation technique for the computation of optic flow and 
         *         egomotion",<br>
         *  journal="Biological Cybernetics",<br>
         *  year="1994",<br>
         *  month="Sep",<br>
         *  day="01",<br>
         *  volume="71",<br>
         *  number="5",<br>
         *  pages="401--415",<br>
         *  issn="1432-0770",<br>
         *  doi="10.1007/BF00198917",<br>
         *  url={https://doi.org/10.1007/BF00198917}<br>
         *  }
         *
         *	@param curr_img pixels of current image
         *	@param last_img pixels of previous image
         *	@param numpix number of pixels
         *	@param scale value of one pixel of motion (for scaling output)
         *	@param out pointer to integer value for output.
         */
        static void ofoIIA_1D(
                uint8_t * curr_img, 
                uint8_t * last_img, 
                uint8_t numpix, 
                uint16_t scale, 
                int16_t *out);

        /**
         *  Runs a two-dimensional version of the Srinivasan algorithm, using a
         *  plus-shaped configuration of pixels
         *
         *	@param curr_img pixels of current image
         *	@param last_img pixels of previous image
         *	@param rows number of rows in image
         *	@param cols number of cols in image
         *	@param scale value of one pixel of motion (for scaling output)
         *	@param ofx pointer to integer value for X shift.
         *	@param ofy pointer to integer value for Y shift.
         */

        void ofoIIA_Plus_2D(
                uint8_t * curr_img, 
                uint8_t * last_img, 
                uint16_t rows, 
                uint16_t cols, 
                uint16_t scale,
                int16_t * ofx,int16_t * ofy);

        /**
         * Same as above, using square configuration
         */
        static void ofoIIA_Square_2D(
                uint8_t * curr_img, 
                uint8_t * last_img, 
                uint16_t rows, 
                uint16_t cols, 
                uint16_t scale,
                int16_t * ofx,
                int16_t * ofy);

        /**
         *	Computes optical flow in plus configuration between two images using
         *	the algorithm desribed in
         *
         * \@inproceedings{Lucas:1981:IIR:1623264.1623280,<br>
         * author = {Lucas, Bruce D. and Kanade, Takeo},<br>
         * title = {An Iterative Image Registration Technique with an Application
         * to Stereo Vision},<br> booktitle = {Proceedings of the 7th International
         * Joint Conference on Artificial Intelligence - Volume 2},<br>
         * series = {IJCAI'81},<br>
         * year = {1981},<br>
         * location = {Vancouver, BC, Canada},<br>
         * pages = {674--679},<br>
         * numpages = {6},<br>
         * url = {http://dl.acm.org/citation.cfm?id=1623264.1623280},<br>
         * acmid = {1623280},<br>
         * publisher = {Morgan Kaufmann Publishers Inc.},<br>
         * address = {San Francisco, CA, USA}<br>
         * }
         *
         *	This algorithm assumes that displacements are generally on the order of
         *	one pixel or less.  This version uses a plus-shaped configuration of
         *	pixels.
         *
         *	@param curr_img pixels of current image
         *	@param last_img pixels of previous image
         *	@param rows number of rows in image
         *	@param cols number of cols in image
         *	@param scale value of one pixel of motion (for scaling output)
         *	@param ofx pointer to integer value for X shift.
         *	@param ofy pointer to integer value for Y shift.
         */
        static void ofoLK_Plus_2D(
                uint8_t * curr_img, 
                uint8_t * last_img, 
                uint16_t rows, uint16_t cols, 
                uint16_t scale, 
                int16_t * ofx, 
                int16_t * ofy);

        /**
         * Same as above, using square pixel configuration
         */
        static void ofoLK_Square_2D(
                uint8_t * curr_img, 
                uint8_t * last_img, 
                uint16_t rows, 
                uint16_t cols, 
                uint16_t scale,
                int16_t * ofx,
                int16_t * ofy);

};

#if 0

void ofoLPF(int16_t *filtered_OF, int16_t *new_OF, float alpha)
{
    (*filtered_OF)=(*filtered_OF)+((float)(*new_OF)-(*filtered_OF))	*alpha;
}

bool ofoAccumulate(int16_t new_OF, int16_t *acc_OF, uint16_t threshold)
{
    bool reset = false;

    if((new_OF>threshold)||(new_OF<-threshold))
    {
        *acc_OF += new_OF;
        reset = true;
    }

    return reset;
}


void ofoIIA_1D(uint8_t * curr_img, uint8_t * last_img, uint16_t	numpix, uint16_t scale, uint16_t *out) 
{
    // Set up pointers
    uint8_t * pleft = curr_img;	    //left-shifted image
    uint8_t * pone = curr_img+1;	//center image
    uint8_t * pright = curr_img+2;	//right-shifted image
    uint8_t * ptwo = last_img+1;	//time-shifted image

    int32_t top    = 0;
    int32_t bottom = 0;

    for (uint8_t i=0; i<numpix-2; ++i) 
    {
        int32_t deltat = (int)(*ptwo) - (int)(*pone);    // temporal gradient i.e. pixel change over time
        int32_t deltax = (int)(*pright) - (int)(*pleft); // spatial gradient i.e. pixel change over space
        top += deltat * deltax;
        bottom += deltax * deltax;

        // Increment pointers
        pleft++;
        pone++;
        pright++;
        ptwo++;
    }

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    *out = 2*top*scale/bottom;
}


void ofoIIA_Plus_2D(uint8_t * curr_img, uint8_t * last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A=0, BD=0, C=0, E=0, F=0;

    // set up pointers
    uint8_t *f0 = curr_img + cols + 1;   // center image
    uint8_t *f1 = curr_img + cols + 2;   // right-shifted image
    uint8_t *f2 = curr_img + cols;       // left-shifted image	
    uint8_t *f3 = curr_img + 2*cols + 1; // down-shifted image	
    uint8_t *f4 = curr_img + 1;		     // up-shifted image
    uint8_t *fz = last_img + cols + 1; 	 // time-shifted image

    // loop through
    for (uint16_t r=1; r<rows-1; ++r) { 

        for (uint16_t c=1; c<cols-1; ++c) { 

            // compute differentials, then increment pointers 
            int16_t F2F1 = (*(f2++) - *(f1++));
            int16_t F4F3 = (*(f4++) - *(f3++));
            int16_t FCF0 = (*(fz++) - *(f0++));

            // update summations
            A += (F2F1 * F2F1);
            BD += (F4F3 * F2F1);
            C  += (FCF0 * F2F1);                   
            E += (F4F3 * F4F3);
            F  += (FCF0 * F4F3);                                   
        }

        f0+=2;	//move to next row of image
        fz+=2;
        f1+=2;
        f2+=2;
        f3+=2;
        f4+=2;
    }

    int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
    int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
    int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = (2*scale*top1)/bottom;
    int64_t YS = (2*scale*top2)/bottom;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}


void ofoIIA_Square_2D(uint8_t * curr_img, uint8_t * last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A=0, BD=0, C=0, E=0, F=0;

    // set up pointers
    uint8_t *f0 = curr_img;             // top left 
    uint8_t *f1 = curr_img + 1; 		// top right
    uint8_t *f2 = curr_img + cols; 	    // bottom left
    uint8_t *f3 = curr_img + cols + 1; 	// bottom right
    uint8_t *fz = last_img; 		    // top left time-shifted

    for (uint16_t r=0; r<rows-1; ++r) { 

        for (uint16_t c=0; c<cols-1; ++c) { 

            // compute differentials
            int16_t F2F1 = ((*(f0) - *(f1)) + (*(f2) - *(f3))) ;
            int16_t F4F3 = ((*(f0) - *(f2)) + (*(f1) - *(f3))) ;
            int16_t FCF0 = (*(fz) - *(f0));

            //increment pointers
            f0++;
            fz++;
            f1++;
            f2++;
            f3++;

            // update summations
            A += (F2F1 * F2F1);
            BD += (F4F3 * F2F1);
            C  += (FCF0 * F2F1);                   
            E += (F4F3 * F4F3);
            F  += (FCF0 * F4F3);                                   
        }

        //go to next row
        f0++;
        fz++;
        f1++;
        f2++;
        f3++;
    }

    int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
    int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
    int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = (2*scale*top1)/bottom;
    int64_t YS = (2*scale*top2)/bottom;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}

void ofoLK_Plus_2D(uint8_t * curr_img, uint8_t * last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
    int16_t  F2F1, F4F3, FCF0;

    // set up pointers
    uint8_t *f0 = curr_img + cols + 1;   // center image
    uint8_t *f1 = curr_img + cols + 2;   // right-shifted image
    uint8_t *f2 = curr_img + cols;       // left-shifted image	
    uint8_t *f3 = curr_img + 2*cols + 1; // down-shifted image	
    uint8_t *f4 = curr_img + 1;		     // up-shifted image
    uint8_t *fz = last_img + cols + 1; 	 // time-shifted image

    for (uint16_t r=1; r<rows-1; ++r) { 

        for (uint16_t c=1; c<cols-1; ++c) { 

            // compute differentials, then increment pointers (post-increment)
            F2F1 = (*(f2++) - *(f1++));	//horizontal differential
            F4F3 = (*(f4++) - *(f3++));	//vertical differential
            FCF0 = (*(fz++) - *(f0++));	//time differential

            // update summations
            A11 += (F2F1 * F2F1);
            A12 += (F4F3 * F2F1);
            A22 += (F4F3 * F4F3);
            b1  += (FCF0 * F2F1);                   
            b2  += (FCF0 * F4F3);                                   
        }
        f0+=2;	//move to next row of image
        fz+=2;
        f1+=2;
        f2+=2;
        f3+=2;
        f4+=2;
    }

    //determinant
    int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = detA == 0 ? 0 : ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * scale / detA;
    int64_t YS = detA == 0 ? 0 : ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * scale / detA;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}

void ofoLK_Square_2D(uint8_t * curr_img, uint8_t * last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;

    // set up pointers
    uint8_t *f0 = curr_img;             // top left 
    uint8_t *f1 = curr_img + 1; 		// top right
    uint8_t *f2 = curr_img + cols; 	    // bottom left
    uint8_t *f3 = curr_img + cols + 1; 	// bottom right
    uint8_t *fz = last_img; 		    // top left time-shifted

    // loop through
    for (uint16_t r=0; r<rows-1; ++r) { 

        for (uint16_t c=0; c<cols-1; ++c) { 

            // compute differentials      
            int16_t F2F1 = ((*(f0) - *(f1)) + (*(f2) - *(f3))) ;
            int16_t F4F3 = ((*(f0) - *(f2)) + (*(f1) - *(f3))) ;
            int16_t FCF0 = (*(fz) - *(f0));

            //increment pointers
            f0++;
            fz++;
            f1++;
            f2++;
            f3++;

            // update summations
            A11 += (F2F1 * F2F1);
            A12 += (F4F3 * F2F1);
            A22 += (F4F3 * F4F3);
            b1  += (FCF0 * F2F1);                   
            b2  += (FCF0 * F4F3);                                   
        }

        //go to next row
        f0++;
        fz++;
        f1++;
        f2++;
        f3++;
    }

    //compute determinant
    int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = detA == 0 ? 0 : ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * scale / detA;
    int64_t YS = detA == 0 ? 0 : ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * scale / detA;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}

#endif
