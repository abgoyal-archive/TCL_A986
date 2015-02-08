

#include <stdio.h>
#include <stdlib.h>

#include "readpng.h"    /* typedefs, common macros, public prototypes */


ulg  width, height;
int  bit_depth, color_type, channels;
uch  *image_data = NULL;
FILE *saved_infile;


void readpng_version_info()
{
    fprintf(stderr, "   Compiled without libpng, zlib or PBMPLUS/NetPBM.\n");
}


/* return value = 0 for success, 1 for bad sig, 2 for bad IHDR, 4 for no mem */

int readpng_init(FILE *infile, ulg *pWidth, ulg *pHeight)
{
    static uch ppmline[256];
    int maxval;


    saved_infile = infile;

    fgets(ppmline, 256, infile);
    if (ppmline[0] != 'P' || ppmline[1] != '6') {
        fprintf(stderr, "ERROR:  not a PPM file\n");
        return 1;
    }
    /* possible color types:  P5 = grayscale (0), P6 = RGB (2), P8 = RGBA (6) */
    if (ppmline[1] == '6') {
        color_type = 2;
        channels = 3;
    } else if (ppmline[1] == '8') {
        color_type = 6;
        channels = 4;
    } else /* if (ppmline[1] == '5') */ {
        color_type = 0;
        channels = 1;
    }

    do {
        fgets(ppmline, 256, infile);
    } while (ppmline[0] == '#');
    sscanf(ppmline, "%lu %lu", &width, &height);

    do {
        fgets(ppmline, 256, infile);
    } while (ppmline[0] == '#');
    sscanf(ppmline, "%d", &maxval);
    if (maxval != 255) {
        fprintf(stderr, "ERROR:  maxval = %d\n", maxval);
        return 2;
    }
    bit_depth = 8;

    *pWidth = width;
    *pHeight = height;

    return 0;
}





int readpng_get_bgcolor(uch *red, uch *green, uch *blue)
{
    return 1;
}




/* display_exponent == LUT_exponent * CRT_exponent */

uch *readpng_get_image(double display_exponent, int *pChannels, ulg *pRowbytes)
{
    ulg  rowbytes;


    /* expand palette images to RGB, low-bit-depth grayscale images to 8 bits,
     * transparency chunks to full alpha channel; strip 16-bit-per-sample
     * images to 8 bits per sample; and convert grayscale to RGB[A] */

    /* GRR WARNING:  grayscale needs to be expanded and channels reset! */

    *pRowbytes = rowbytes = channels*width;
    *pChannels = channels;

    if ((image_data = (uch *)malloc(rowbytes*height)) == NULL) {
        return NULL;
    }

    Trace((stderr, "readpng_get_image:  rowbytes = %ld, height = %ld\n", rowbytes, height));


    /* now we can go ahead and just read the whole image */

    fread(image_data, 1L, rowbytes*height, saved_infile);


    return image_data;
}


void readpng_cleanup(int free_image_data)
{
    if (free_image_data && image_data) {
        free(image_data);
        image_data = NULL;
    }
}
