

#import <Cocoa/Cocoa.h>

@interface DumpRenderTreeDraggingInfo : NSObject <NSDraggingInfo> {
@private
    NSSize offset;
    NSImage *draggedImage;
    NSPasteboard *draggingPasteboard;
    id draggingSource;
}

- (id)initWithImage:(NSImage *)image offset:(NSSize)offset pasteboard:(NSPasteboard *)pasteboard source:(id)source;

- (NSWindow *)draggingDestinationWindow;
- (NSDragOperation)draggingSourceOperationMask;
- (NSPoint)draggingLocation;
- (NSPoint)draggedImageLocation;
- (NSImage *)draggedImage;
- (NSPasteboard *)draggingPasteboard;
- (id)draggingSource;
- (int)draggingSequenceNumber;

- (void)slideDraggedImageTo:(NSPoint)screenPoint;
- (NSArray *)namesOfPromisedFilesDroppedAtDestination:(NSURL *)dropDestination;
@end

