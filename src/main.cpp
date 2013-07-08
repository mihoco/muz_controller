#include "testApp.h"
#include "ofAppGlutWindow.h"
#include "ofMain.h"

int main() {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1024, 768, OF_WINDOW);
	ofRunApp(new testApp());
}
