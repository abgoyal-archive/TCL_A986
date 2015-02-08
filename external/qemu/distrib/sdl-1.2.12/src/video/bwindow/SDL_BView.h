
#include "SDL_config.h"

#ifndef _SDL_BView_h
#define _SDL_BView_h

/* This is the event handling and graphics update portion of SDL_BWin */

extern "C" {
#include "../../events/SDL_events_c.h"
};

class SDL_BView : public BView
{
public:
	SDL_BView(BRect frame) : 
		BView(frame, "SDL View", B_FOLLOW_ALL_SIDES,
					(B_WILL_DRAW|B_FRAME_EVENTS)) {
		image = NULL;
		xoff = yoff = 0;
		SetViewColor(0,0,0,0);
		SetHighColor(0,0,0,0);
	}
	virtual ~SDL_BView() {
		SetBitmap(NULL);
	}
	/* Set drawing offsets for fullscreen mode */
	virtual void SetXYOffset(int x, int y) {
		xoff = x;
		yoff = y;
	}
	virtual void GetXYOffset(int &x, int &y) {
		x = xoff;
		y = yoff;
	}
	virtual void GetXYOffset(float &x, float &y) {
		x = (float)xoff;
		y = (float)yoff;
	}
	/* The view changed size. If it means we're in fullscreen, we
	 * draw a nice black box in the entire view to get black borders.
	 */
	virtual void FrameResized(float width, float height) {
		BRect bounds;
		bounds.top = bounds.left = 0;
		bounds.right = width;
		bounds.bottom = height;
		/* Fill the entire view with black */ 
		FillRect(bounds, B_SOLID_HIGH);
		/* And if there's an image, redraw it. */
		if( image ) {
			bounds = image->Bounds();
			Draw(bounds);
		}
	}
	
	/* Drawing portion of this complete breakfast. :) */
	virtual void SetBitmap(BBitmap *bitmap) {
		if ( image ) {
			delete image;
		}
		image = bitmap;
	}
	virtual void Draw(BRect updateRect) {
		if ( image ) {
			if(xoff || yoff) {	
				BRect dest;
				dest.top    = updateRect.top + yoff;
				dest.left   = updateRect.left + xoff;
				dest.bottom = updateRect.bottom + yoff;
				dest.right  = updateRect.right + xoff;
				DrawBitmap(image, updateRect, dest);
			} else {
				DrawBitmap(image, updateRect, updateRect);
			}
		}
	}
	virtual void DrawAsync(BRect updateRect) {
		if(xoff || yoff) {	
			BRect dest;
			dest.top    = updateRect.top + yoff;
			dest.left   = updateRect.left + xoff;
			dest.bottom = updateRect.bottom + yoff;
			dest.right  = updateRect.right + xoff;;
			DrawBitmapAsync(image, updateRect, dest);
		} else {
			DrawBitmapAsync(image, updateRect, updateRect);
		}
	}

private:
	BBitmap *image;
	int xoff, yoff;
};

#endif /* _SDL_BView_h */
