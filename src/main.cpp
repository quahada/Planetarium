#include "testApp.h"
#include "ofAppGlutWindow.h"

int main() {
	ofAppGlutWindow window;
	//ofSetupOpenGL(&window, 1290, 480, OF_WINDOW);
	ofSetupOpenGL(&window, 640, 480, OF_WINDOW);
	ofRunApp(new testApp());
}
