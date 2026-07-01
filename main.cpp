#include "Application.h"

const unsigned int WIDTH = 1280;
const unsigned int HEIGHT = 800;

int main() {
	Application app(WIDTH, HEIGHT, "Circle Loop Drawing");
	app.run();
	return 0;
}