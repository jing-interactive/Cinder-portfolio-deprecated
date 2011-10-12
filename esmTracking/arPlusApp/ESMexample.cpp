//#============================================================================
//#
//#       Filename:  ESMexample.c
//#
//#    Description:  An example program for the ESMlibrary
//#
//#            $Id: ESMexample.c,v 1.11 2005/07/01 06:04:58 sbenhima Exp $
//#
//#         Author:  Selim BENHIMANE & Ezio MALIS
//#        Company:  INRIA, Sophia-Antipolis, FRANCE
//#          Email:  Selim.Benhimane@inria.fr
//#          Email:  Ezio.Malis@inria.fr
//#
//#        Project:  ICARE
//#
//#===========================================================================

// #####   HEADER FILE INCLUDES   #############################################

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include "ESMlibry.h"

// #####   MACROS  -  LOCAL TO THIS SOURCE FILE   #############################

#define MAXFILENAME 100

// #####   DATA TYPES  -  LOCAL TO THIS SOURCE FILE   #########################

// #####   TYPE DEFINITIONS  -  LOCAL TO THIS SOURCE FILE   ###################

// #####   VARIABLES  -  LOCAL TO THIS SOURCE FILE   ##########################

// #####   PROTOTYPES  -  LOCAL TO THIS SOURCE FILE   #########################

void DrawLine (imageStruct *image, int r1, int c1, int r2, int c2);
void DrawResult (int sx, int sy, float H[9], imageStruct I);

// #####   FUNCTION DEFINITIONS  -  EXPORTED FUNCTIONS   ######################

int main (int argc, char **argv)
{
  unsigned int i, nbimages = 10;
  char filename[MAXFILENAME];

  // The tracking parameters
  // miter: the number of iterations of the ESM algorithm (>= 1);
  // mprec: the precision of the ESM algorithm (1..10)
  // low precision = 1, high precision = 10;
  // miter and mprec should be chosen depending on the available 
  // computation time; 
  // For low-speed PCs or high video framerate choose low miter and low mprec.
  int miter = 5,  mprec = 2;    

  // The window position (upper left corner)
  // The image coordinates start from (0,0)
  int posx = 200, posy = 250;

  // The window size
  int sizx = 100, sizy = 120;

  // The image read / acquired 
  imageStruct I; 

  // The global tracking structure
  trackStruct T; 
  
  // Read the reference image
  if (ReadPgm ("im000.pgm", &I))
    return (1);
  else
    printf ("ESM Reading the reference image...\n");
     
  // Memory allocation for the tracking
  if (MallTrack (&T, &I, posx, posy, sizx, sizy, miter, mprec))
    return (1);
  else
    printf ("ESM Tracking structure ready\n");
  
  // Save the reference pattern
  sprintf (filename, "patr.pgm");
  if (SavePgm (filename, GetPatr (&T)))
    return (1);
  else
    printf ("ESM Writing %s\n", filename);

  for (i = 0; i < nbimages; i++) {     
    // Read the current image
    sprintf (filename, "im%03d.pgm", i);
    if (ReadPgm (filename, &I)) 
      return (1);
    else
      printf ("ESM Reading %s\n", filename);
    
    // Perform the tracking
    if (MakeTrack (&T, &I))
      return (1);
    
    // Draw a black window around the pattern
    // T.homog contains the pointer to the homography matrix
    DrawResult (sizx, sizy, T.homog, I);    

    // Save the the current image 
    sprintf (filename, "res_im%03d.pgm", i);
    if (SavePgm (filename, &I))
      return (1);
    else
      printf ("ESM Writing %s\n", filename);

    // Save the reprojection of the current pattern
    sprintf (filename, "patc%03d.pgm", i);
    if (SavePgm (filename, GetPatc (&T)))
      return (1);
    else
      printf ("ESM Writing %s\n", filename);
  }
  
  printf ("The final homography is: \n");
  printf ("%.3f\t%.3f\t%.3f\n", T.homog[0], T.homog[1], T.homog[2]);
  printf ("%.3f\t%.3f\t%.3f\n", T.homog[3], T.homog[4], T.homog[5]);
  printf ("%.3f\t%.3f\t%.3f\n", T.homog[6], T.homog[7], T.homog[8]);

  // Free the global structure memory
  FreeTrack (&T);
  
  getch ();
  return (0);
}

// #####   FUNCTION DEFINITIONS  -  LOCAL TO THIS SOURCE FILE   ###############

void DrawLine (imageStruct *image, int r1, int c1, int r2, int c2)
{
  int dr, dc, temp;
  int cols = image->cols, rows = image->rows;
  int i, point, area;
  
  area = cols * rows;
  if (r1 == r2 && c1 == c2) 
    return;
  if (abs (r2 - r1) < abs (c2 - c1)) {
    if (c1 > c2) {
      temp = r1; r1 = r2; r2 = temp;
      temp = c1; c1 = c2; c2 = temp;
    }
    dr = r2 - r1;
    dc = c2 - c1;
    temp = (r1 - c1 * dr / dc)*cols;
    for (i = c1; i <= c2; i++) {
      point = temp + (i * dr) / dc * cols  + i;
      if ((point >= 0) & (point < area))  
	image->data[point] = 0.0;
    }
  } 
  else {
    if (r1 > r2) {
      temp = r1; r1 = r2; r2 = temp;
      temp = c1; c1 = c2; c2 = temp;
    }
    dr = r2 - r1;
    dc = c2 - c1;
    temp =  c1 - r1 * dc / dr;
    for (i = r1; i <= r2; i++) {
      point = temp + i*cols + (i * dc) / dr;
      if ((point >= 0) & (point < area))  
	image->data[point] = 0.0;
    }
  }

  return;
}

void DrawResult (int sx, int sy, float H[9], imageStruct I) 
{
  int i;
  float pEnd[8], pIni[8];

  pIni[0] =        0.0;    pIni[1] =          0.0;  pIni[2] = (float) sx-1;  pIni[3] =          0.0;
  pIni[4] = (float) sx-1;  pIni[5] = (float) sy-1;  pIni[6] =          0.0;  pIni[7] = (float) sy-1;
  for (i = 0; i < 4; i++) {
    pEnd[2*i] = (H[0]*pIni[2*i] + H[1]*pIni[2*i+1] + H[2])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]); 
    pEnd[2*i+1] = (H[3]*pIni[2*i] + H[4]*pIni[2*i+1] + H[5])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);      
  }
  DrawLine (&I, (int)pEnd[1], (int)pEnd[0], (int)pEnd[3], (int)pEnd[2]);
  DrawLine (&I, (int)pEnd[3], (int)pEnd[2], (int)pEnd[5], (int)pEnd[4]);
  DrawLine (&I, (int)pEnd[5], (int)pEnd[4], (int)pEnd[7], (int)pEnd[6]);
  DrawLine (&I, (int)pEnd[7], (int)pEnd[6], (int)pEnd[1], (int)pEnd[0]);

  return;
}


