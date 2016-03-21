/*
   Taken From
   http://web.engr.oregonstate.edu/~mjb/cs553/bmptotexture.cpp
*/

#include <cstdio>

unsigned char* BmpToTexture( char *, int *, int * );
int ReadInt( FILE * );
short ReadShort( FILE * );