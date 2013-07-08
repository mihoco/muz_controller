//
//  makeFloatingWindow.cpp
//  ofxKinectExample
//
//  Created by Mihoko Abe on 4/02/13.
//
//

#include "makeFloatingWindow.h"
#import <Cocoa/cocoa.h>

void makeFloatingWindow()
{
    NSWindow *window = [NSApp mainWindow];
    
    if(window)
    {
       // NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
        [window setLevel:NSFloatingWindowLevel];
       // [pool release];
    }
}
