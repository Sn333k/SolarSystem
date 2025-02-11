#define _USE_MATH_DEFINES
#include <windows.h>
#include <gl/gl.h>
#include <gl/glut.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cstdlib>

using namespace std;
float days = 0;
float speed = 0.00;
struct Vertex {
	float x1, y1, z1;
	float x2, y2, z2;
	float u, v1, v2;
	float x1n, y1n, z1n;
	float x2n, y2n, z2n;
};
struct Sphere {
	float x = 0.0, y = 0.0, z = 0.0, angle = 0.0;
	float angleOrbit = 1.0;
	float orbitalRad = 10.0;
	float odchylenie = 0.02;
	float radius = 0.0;
	int timeOfOrbiting = 10;
	int timeOfDay = 10;
	float mimosrod = 0.01;
	vector<vector<Vertex>> vertexes;
	GLuint texture;
};

//Pozycjonowanie kamery
float camPos[3] = { 50.0f, 50.0f, 50.0f };// X Y Z
float centerX = 0.0f; //Obserwowany punkt X
float centerY = 0.0f; //Obserwowany punkt Y
float centerZ = 0.0f; //Obserwowany punkt Z
float camAngleX = 0.0f;  // Kat obrotu kamery woko³ osi X
float camAngleY = 0.0f;  // Kat obrotu kamery woko³ osi Y
float camDist = 100.0f; // Odleg³osc kamery od center wykorzystywana do obliczen

//Pozycjonowanie zrodel swiatla
float lightDist = 150.0f; // Odleg³osc kamery od center wykorzystywana do obliczen
float light0Angle[2] = { 0.0f, 0.0f };   // K¹t obrotu œwiat³a 1
float light0Pos[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

//Obsluga myszy
int lastMouseX, lastMouseY; // Pozycje myszy
bool leftMouseButtonActive = false; //Wcisniety lewy przycisk
bool rightMouseButtonActive = false; //Wcisniety prawy przycisk

//Wyswietlany obiekt
int N = 50; // Liczba segmentow w poziomie i pionie 
int currentObject = 0;
int numberOfPlantes = 10;

//Tekstur
int currentTexture = 2; //1- struktura(cegly) 2-posrednia(kamienie) 3-bezstruktury(piach)
GLint ImWidth, ImHeight, ImComponents;
GLenum ImFormat;

vector<Sphere> solarSystem;
vector<GLuint> textures;
const char* tgaFiles[] = { "xsun.tga", "xmercury.tga", "xvenus.tga", "xearth.tga", "xmars.tga", "xjupiter.tga", "xsaturn.tga", "xuranus.tga", "xneptune.tga", "xmoon.tga" };
GLuint saturnring;

void setMaterial();
void setSunMaterial();
GLbyte* LoadTGAImage(const char* FileName, GLint* ImWidth, GLint* ImHeight, GLint* ImComponents, GLenum* ImFormat) {
#pragma pack(1)           
	typedef struct
	{
		GLbyte    idlength;
		GLbyte    colormaptype;
		GLbyte    datatypecode;
		unsigned short    colormapstart;
		unsigned short    colormaplength;
		unsigned char     colormapdepth;
		unsigned short    x_orgin;
		unsigned short    y_orgin;
		unsigned short    width;
		unsigned short    height;
		GLbyte    bitsperpixel;
		GLbyte    descriptor;
	}TGAHEADER;
#pragma pack(8)

	FILE* pFile;
	TGAHEADER tgaHeader;
	unsigned long lImageSize;
	short sDepth;
	GLbyte* pbitsperpixel = NULL;
	*ImWidth = 0;
	*ImHeight = 0;
	*ImFormat = GL_BGR_EXT;
	*ImComponents = GL_RGB8;
#pragma warning(suppress : 4996)
	pFile = fopen(FileName, "rb");
	if (pFile == NULL)
		return NULL;
	fread(&tgaHeader, sizeof(TGAHEADER), 1, pFile);
	*ImWidth = tgaHeader.width;
	*ImHeight = tgaHeader.height;
	sDepth = tgaHeader.bitsperpixel / 8;
	if (tgaHeader.bitsperpixel != 8 && tgaHeader.bitsperpixel != 24 && tgaHeader.bitsperpixel != 32)
		return NULL;
	lImageSize = tgaHeader.width * tgaHeader.height * sDepth;
	pbitsperpixel = (GLbyte*)malloc(lImageSize * sizeof(GLbyte));
	if (pbitsperpixel == NULL)
		return NULL;

	if (fread(pbitsperpixel, lImageSize, 1, pFile) != 1) {
		free(pbitsperpixel);
		return NULL;
	}
	switch (sDepth) {
	case 3:
		*ImFormat = GL_BGR_EXT;
		*ImComponents = GL_RGB8;
		break;
	case 4:
		*ImFormat = GL_BGRA_EXT;
		*ImComponents = GL_RGBA8;
		break;
	case 1:
		*ImFormat = GL_LUMINANCE;
		*ImComponents = GL_LUMINANCE8;
		break;
	};
	fclose(pFile);
	return pbitsperpixel;
}
void printPlanetInfo(Sphere planet) {
	cout << "angle " << planet.angle << endl;
	cout << "angleOrbit " << planet.angleOrbit << endl;
	cout << "orbitalRad " << planet.orbitalRad << endl;
	cout << "timeOfOrbiting " << planet.timeOfOrbiting << endl;
}

void generateSphere(float radius, Sphere& sphere, float orbitLgh, int timeOfOrbiting, int timeOfDay, int x, int y, int z, float nachylenie, float mimosrod) {
	sphere.angleOrbit = 0.0;
	sphere.x = x;
	sphere.y = y;
	sphere.z = z;
	sphere.orbitalRad = orbitLgh;
	sphere.timeOfOrbiting = timeOfOrbiting;
	sphere.vertexes.resize(N);
	sphere.odchylenie = nachylenie;
	sphere.mimosrod = mimosrod;
	sphere.timeOfDay = timeOfDay;
	sphere.radius = radius;
	for (int i = 0; i < N; ++i) {
		sphere.vertexes[i].resize(N + 1);
		// K¹t pionowy (theta) dla dwóch s¹siednich pasm
		float theta1 = i * M_PI / N;
		float theta2 = (i + 1) * M_PI / N;

		for (int j = 0; j <= N; ++j) {
			// K¹t poziomy (phi) dla punktów w danym pasmie
			float phi = j * 2 * M_PI / N;

			// Oblicz wspó³rzêdne wierzcho³ków
			sphere.vertexes[i][j].x1n = sin(theta1) * cos(phi);
			sphere.vertexes[i][j].x1 = radius * sphere.vertexes[i][j].x1n;
			sphere.vertexes[i][j].y1n = cos(theta1);
			sphere.vertexes[i][j].y1 = radius * sphere.vertexes[i][j].y1n;
			sphere.vertexes[i][j].z1n = sin(theta1) * sin(phi);
			sphere.vertexes[i][j].z1 = radius * sphere.vertexes[i][j].z1n;

			sphere.vertexes[i][j].x2n = sin(theta2) * cos(phi);
			sphere.vertexes[i][j].x2 = radius * sphere.vertexes[i][j].x2n;
			sphere.vertexes[i][j].y2n = cos(theta2);
			sphere.vertexes[i][j].y2 = radius * sphere.vertexes[i][j].y2n;
			sphere.vertexes[i][j].z2n = sin(theta2) * sin(phi);
			sphere.vertexes[i][j].z2 = radius * sphere.vertexes[i][j].z2n;

			// Wspó³rzêdne tekstury UV
			sphere.vertexes[i][j].u = 1.0 - phi / (2 * M_PI);
			sphere.vertexes[i][j].v1 = 1.0 - theta1 / M_PI;
			sphere.vertexes[i][j].v2 = 1.0 - theta2 / M_PI;
		}
	}
}
void drawSphere(Sphere sphere) {
	glBindTexture(GL_TEXTURE_2D, sphere.texture);
	glPushMatrix();
	//float delta = (float(days % sphere.timeOfOrbiting)) / sphere.timeOfOrbiting;
	//cout << days % sphere.timeOfOrbiting<<" ";
	//cout << sphere.orbitLgh * cos(delta * M_PI / 180.0f) << endl;
	//cout << sphere.orbitLgh * sin(delta * M_PI / 180.0f) << endl;
	glTranslatef(sphere.x, sphere.y, sphere.z);  // Przesuniêcie po osi 
	glRotatef(sphere.angle, 0.0f, 1.0f, 0.0f); // obórt wokol osi Y
	glRotatef(sphere.odchylenie, 1.0f, 0.0f, 0.0f); // odchylenie
	
	

	for (int i = 0; i < N; ++i) {
		glBegin(GL_TRIANGLE_STRIP);
		for (int j = 0; j <= N; ++j) {
			glNormal3f(sphere.vertexes[i][j].x1n, sphere.vertexes[i][j].y1n, sphere.vertexes[i][j].z1n);
			glTexCoord2f(sphere.vertexes[i][j].u, sphere.vertexes[i][j].v1);
			glVertex3f(sphere.vertexes[i][j].x1, sphere.vertexes[i][j].y1, sphere.vertexes[i][j].z1);

			glNormal3f(sphere.vertexes[i][j].x2n, sphere.vertexes[i][j].y2n, sphere.vertexes[i][j].z2n);
			glTexCoord2f(sphere.vertexes[i][j].u, sphere.vertexes[i][j].v2);
			glVertex3f(sphere.vertexes[i][j].x2, sphere.vertexes[i][j].y2, sphere.vertexes[i][j].z2);
		}
		glEnd();
	}
	glPopMatrix();
}
void drawOrbit(float semiMajorAxis, float eccentricity) {
	// Liczba punktów na orbicie
	const int numPoints = 100;

	// Rysowanie orbity

	glBegin(GL_LINE_LOOP);
	for (int i = 0; i < numPoints; i++) {
		float angle = 2 * M_PI * i / numPoints;  // Obliczamy k¹t dla ka¿dego punktu

		// Równanie elipsy dla wspó³rzêdnych x i z
		float r = semiMajorAxis * (1 - eccentricity * eccentricity) / (1 + eccentricity * cos(angle));

		// Wspó³rzêdne elipsy w 3D (orbita na p³aszczyŸnie XZ)
		float x = r * cos(angle);
		float z = r * sin(angle);
		glColor3f(0.1, 0.1, 0.1);
		glVertex3f(x, 0.0f, z);  // Zwracamy wspó³rzêdne punktu na orbicie
	}
	glEnd();
}
void drawMoonOrbit(float earthX, float earthZ, float moonOrbitRadius) {
	const int numPoints = 100;
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i < numPoints; i++) {
		float angle = 2 * M_PI * i / numPoints;
		float moonX = earthX + moonOrbitRadius * cos(angle);  // Ksiê¿yc na orbicie wokó³ Ziemi
		float moonZ = earthZ + moonOrbitRadius * sin(angle);
		glVertex3f(moonX, 0.0f, moonZ);  // Rysowanie orbity Ksiê¿yca
	}
	glEnd();
}

void setTexture() {
	GLuint textureTMP;
	for (int i = 0; i < numberOfPlantes; i++) {
		glGenTextures(1, &textureTMP);
		glBindTexture(GL_TEXTURE_2D, textureTMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		GLbyte* pBytes = LoadTGAImage(tgaFiles[i], &ImWidth, &ImHeight, &ImComponents, &ImFormat);
		if (pBytes) {
			glTexImage2D(GL_TEXTURE_2D, 0, ImComponents, ImWidth, ImHeight, 0, ImFormat, GL_UNSIGNED_BYTE, pBytes);
			free(pBytes);
		}
		textures.push_back(textureTMP);
	}
}
void setLight() {
	GLfloat light_position[] = { 0.0f, 0.0f, 0.0f, 1.0f };  // Pozycja œwiat³a w centrum
	GLfloat ambient_light[] = { 0.2f, 0.2f, 0.2f, 1.0f };  // Œwiat³o ambient
	GLfloat diffuse_light[] = { 1.0f, 1.0f, 0.9f, 1.0f };  // Œwiat³o diffuse
	GLfloat specular_light[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // Œwiat³o specular

	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_light);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_light);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular_light);

	// Brak wygasania œwiat³a
	glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1f);
	glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.0f);
	glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.0f);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}
void setSunMaterial() {
	GLfloat ambient[] = { 1.0f, 0.8f, 0.0f, 1.0f };
	GLfloat diffuse[] = { 1.0f, 0.8f, 0.0f, 1.0f };
	GLfloat specular[] = { 1.0f, 1.0f, 0.8f, 1.0f };
	GLfloat shininess = 100.0f;

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
}
void setMaterial() {
	GLfloat matAmbient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat matDiffuse[] = { 0.8f, 0.6f, 0.4f, 1.0f };
	GLfloat matSpecular[] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat matShininess[] = { 10.0f };

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, matAmbient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, matDiffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpecular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShininess);
}
void setPlanet() {

	Sphere sun;
	Sphere mercury;
	Sphere venus;
	Sphere earth;
	Sphere mars;
	Sphere jupiter;
	Sphere saturn;
	Sphere uran;
	Sphere neptun;
	Sphere moon;
	//Kolejnosc argumentow  (promien | sfera | dlugosc orbity | czas orbitowania | czas obrotu w godzinach |  x | y | z | odchylenie | mimosrod)
	generateSphere(			3.0,		sun,		0.0001,			1,				609, 0, 0, 0, 0.01, 0.000001);
	generateSphere(			0.1,		mercury,	5,				87,				1407, 5, 0, 0, 0.02, 0.20563);
	generateSphere(			0.4,		venus,		10,				224,			5832, 10, 0, 0, 2.38, 0.006772);
	generateSphere(			0.41,		 earth,		15,				365,			23, 15, 0, 0, 23.26, 0.0167086);
	generateSphere(			0.5,		mars,		22,				686,			23, 22, 0, 0, 25.11, 0.0934);
	generateSphere(			2.0,		jupiter,	32,				4333,			10, 32, 0, 0, 3.07, 0.0489);
	generateSphere(			1.9,		saturn,		40,				10756,			10, 40, 0, 0, 26.44, 0.0565);
	generateSphere(			1.2,		uran,		47,				30707,			17, 47, 0, 0, 82.14, 0.046381);
	generateSphere(			1.1,		neptun,		55,				60223,			16, 55, 0, 0, 29.35, 0.009456);
	generateSphere(			0.1,		moon,		1,				27,				27, 16, 0, 0, 83.18, 0.0549);


	sun.texture = textures[0];
	mercury.texture = textures[1];
	venus.texture = textures[2];
	earth.texture = textures[3];
	mars.texture = textures[4];
	jupiter.texture = textures[5];
	saturn.texture = textures[6];
	uran.texture = textures[7];
	neptun.texture = textures[8];
	moon.texture = textures[9];

	solarSystem.push_back(sun);
	solarSystem.push_back(mercury);
	solarSystem.push_back(venus);
	solarSystem.push_back(earth);
	solarSystem.push_back(mars);
	solarSystem.push_back(jupiter);
	solarSystem.push_back(saturn);
	solarSystem.push_back(uran);
	solarSystem.push_back(neptun);
	solarSystem.push_back(moon);
}

void updateMoon(float daysToAdd) {
	// Obliczanie orbity Ksiê¿yca wokó³ Ziemi
	float moonOrbitRadius = 1.0f;  // Przyk³adowa odleg³oœæ Ksiê¿yca od Ziemi
	float moonAngle = 2 * M_PI * float(days) / 27.3f;  // Okres obrotu Ksiê¿yca wokó³ Ziemi (27.3 dnia)

	// Obliczanie pozycji Ksiê¿yca wzglêdem Ziemi
	float moonX = solarSystem[3].x + moonOrbitRadius * cos(moonAngle);  // Pozycja Ksiê¿yca X
	float moonZ = solarSystem[3].z + moonOrbitRadius * -sin(moonAngle);  // Pozycja Ksiê¿yca Z

	// Aktualizacja pozycji Ksiê¿yca (zak³adaj¹c, ¿e Ksiê¿yc jest w solarSystem[9])
	solarSystem[9].x = moonX;
	solarSystem[9].z = moonZ;

	// Synchronizacja rotacji: k¹t obrotu Ksiê¿yca wokó³ w³asnej osi jest równy jego k¹towi na orbicie
	solarSystem[9].angle = moonAngle * (180.0f / M_PI);  // Zamiana k¹ta na stopnie

	// Utrzymanie k¹ta obrotu Ksiê¿yca w przedziale [0, 360)
	if (solarSystem[9].angle >= 360.0f) {
		solarSystem[9].angle -= 360.0f;
	}
	else if (solarSystem[9].angle < 0.0f) {
		solarSystem[9].angle += 360.0f;
	}
}
void updatePlantes(float daysToAdd, vector<Sphere>& planets) {
	days += daysToAdd;
	for (int planet = 0; planet < numberOfPlantes - 1; planet++) {
		// Obliczanie anomalii œredniej
		float meanAnomaly = 2 * M_PI * (float(days) / solarSystem[planet].timeOfOrbiting);

		// Parametry orbity
		float eccentricity = solarSystem[planet].mimosrod; // Mimosród
		float semiMajorAxis = solarSystem[planet].orbitalRad; // Pó³osie wielka (œredni promieñ orbity)

		// Obliczanie anomalii ekscentrycznej (metoda Newtona-Raphsona)
		float E = meanAnomaly;  // Pocz¹tkowe przybli¿enie
		for (int i = 0; i < 10; i++) {  // Iteracje w celu uzyskania dok³adnoœci
			E = E - (E - eccentricity * sin(E) - meanAnomaly) / (1 - eccentricity * cos(E));
		}

		// Obliczanie k¹ta orbitalnego (true anomaly)
		float theta = atan2(sqrt(1 - eccentricity * eccentricity) * sin(E), cos(E) - eccentricity);

		// Obliczanie odleg³oœci planety na orbicie eliptycznej
		float r = semiMajorAxis * (1 - eccentricity * eccentricity) / (1 + eccentricity * cos(theta));

		// Pozycja planety w przestrzeni 3D (na orbicie eliptycznej)
		solarSystem[planet].x = r * cos(theta);  // Wspó³rzêdna X
		solarSystem[planet].z = -r * sin(theta);  // Wspó³rzêdna Z (zmiana znaku, aby zmieniæ kierunek obrotu)

		// Obliczanie prêdkoœci obrotu planety wokó³ w³asnej osi
		float rotationSpeed = (8640.0 / planets[planet].timeOfDay);
		planets[planet].angle += rotationSpeed * daysToAdd;

		// Utrzymanie k¹ta obrotu w przedziale [0, 360)
		if (planets[planet].angle >= 360.0f) {
			planets[planet].angle -= 360.0f;
		}
		else if (planets[planet].angle < 0.0f) {
			planets[planet].angle += 360.0f;
		}
	}
	glutPostRedisplay();
}
void updateCameraPosition() {
	camPos[0] = camDist * sin(camAngleX) * cos(camAngleY) + solarSystem[currentObject].x;
	camPos[1] = camDist * sin(camAngleY) + solarSystem[currentObject].y;
	camPos[2] = camDist * cos(camAngleX) * cos(camAngleY) + solarSystem[currentObject].z;
}
void timer(int value) {
	updatePlantes(speed, solarSystem);  // Dodajemy 1 dzieñ w ka¿dej klatce animacji
	updateMoon(speed);
	updateCameraPosition();
	// Ponownie wywo³aj timer co 16 ms (~60 FPS)
	glutTimerFunc(16, timer, 0);

}
void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	gluLookAt(camPos[0], camPos[1], camPos[2],
		solarSystem[currentObject].x, solarSystem[currentObject].y, solarSystem[currentObject].z,
		0.0, 1.0, 0.0);
	glLightfv(GL_LIGHT0, GL_POSITION, light0Pos);
	setSunMaterial();

	drawSphere(solarSystem[0]);
	setMaterial();
	for (int planet = 1; planet < numberOfPlantes; planet++) {
		drawSphere(solarSystem[planet]);
		drawOrbit(solarSystem[planet].orbitalRad, solarSystem[planet].mimosrod);
	}
	drawSphere(solarSystem[9]);
	drawMoonOrbit(solarSystem[3].x, solarSystem[3].z, 1.0);

	glFlush();
	glutSwapBuffers();
}
void init() {
	glClearColor(0.0, 0.0, 0.0, 1.0);

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glShadeModel(GL_SMOOTH);
	glCullFace(GL_FRONT);
	glEnable(GL_CULL_FACE);

	setTexture();
	setPlanet();

	glEnable(GL_DEPTH_TEST);
	updatePlantes(1, solarSystem);
	updateMoon(1);
	camPos[0] = camDist * sin(camAngleY);
	camPos[1] = camDist * sin(camAngleX);  // Ruch w górê/dó³
	camPos[2] = camDist * cos(camAngleY) * cos(camAngleX);
	setLight();
}

void handleMouseMotion(int x, int y) {
	int dx = x - lastMouseX;
	int dy = y - lastMouseY;

	if (leftMouseButtonActive) {
		camAngleX -= dx * 0.02f;
		camAngleY += dy * 0.02f;

		float maxAngleY = M_PI_2 - 0.01f;  // Maksymalny k¹t (blisko 90 stopni, ale nie dok³adnie)
		float minAngleY = -M_PI_2 + 0.01f;

		if (camAngleY > maxAngleY) {
			camAngleY = maxAngleY;
		}
		else if (camAngleY < minAngleY) {
			camAngleY = minAngleY;
		}
		updateCameraPosition();
	}

	if (rightMouseButtonActive) {
		float directionX = centerX - camPos[0];
		float directionY = centerY - camPos[1];
		float directionZ = centerZ - camPos[2];
		float magnitude = sqrt(directionX * directionX + directionY * directionY + directionZ * directionZ);

		float normalizedDirectionX = directionX / magnitude;
		float normalizedDirectionY = directionY / magnitude;
		float normalizedDirectionZ = directionZ / magnitude;

		camDist -= dy * 0.1f;
		if (camDist < (2*solarSystem[currentObject].radius + 1.0)) {
			camDist =(2*solarSystem[currentObject].radius + 1.0);
		}
		updateCameraPosition();
	}
	lastMouseX = x;
	lastMouseY = y;
	glutPostRedisplay();
}
void handleMouseClick(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON) {
		leftMouseButtonActive = (state == GLUT_DOWN);
		//cout << rotateCamera << " " << rotateLight1 << " " << rotateLight2 << " " << endl;
	}
	if (button == GLUT_RIGHT_BUTTON) {
		rightMouseButtonActive = (state == GLUT_DOWN);
	}
	if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN) {
		camPos[0] = 0.0f;
		camPos[1] = 0.0f;
		camPos[2] = 100.0f;
		centerX = 0.0f;
		centerY = 0.0f;
		centerZ = 0.0f;
		//std::cout << camPosX << " " << camPosY << " " << camPosZ << "\n";
	}
	lastMouseX = x;
	lastMouseY = y;
	glutPostRedisplay();
}
void handleKeypress(unsigned char key, int x, int y) {
	switch (key)
	{
	case '0': currentObject = 0; camDist = 16; break;
	case '1': currentObject = 1; camDist = 5; break;
	case '2': currentObject = 2; camDist = 5; break;
	case '3': currentObject = 3; camDist = 5; break;
	case '4': currentObject = 4; camDist = 5; break;
	case '5': currentObject = 5; camDist = 8; break;
	case '6': currentObject = 6; camDist = 8; break;
	case '7': currentObject = 7; camDist = 6; break;
	case '8': currentObject = 8; camDist = 6; break;
	case 'y': speed += 0.000694; break;
	case 'h': speed -= 0.000694; break;
	case 'u': speed += 0.041; break;
	case 'j': speed -= 0.041; break;
	case 'i': speed += 1.0001; break;
	case 'k': speed -= 1.0001; break;
	case 'o': speed += 12.0001; break;
	case 'l': speed -= 12.0001; break;

		break;
	}
	updateCameraPosition();
	if (speed < 0) {
		speed = 0;
	}
	glutPostRedisplay();
}

void reshape(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (float)w / (float)h, 1.0, 200.0);
	glMatrixMode(GL_MODELVIEW);
}
void setConsolePositionAndSize(int x, int y, int width, int height) {
	HWND console = GetConsoleWindow();
	if (console) {
		MoveWindow(console, x, y, width, height, TRUE);
	}
}
int main(void) {
	setConsolePositionAndSize(1200, 0, 500, 400);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1200, 1000);
	glutCreateWindow("Uklad Sloneczny 272905");

	std::cout << "==============================================\n";
	std::cout << "|          PRZYPISANIE RUCHOW MYSZY          |\n";
	std::cout << "==============================================\n";
	std::cout << "| Wcisnij lewy aby ruszac kamera             |\n";
	std::cout << "==============================================\n";
	std::cout << "|            PRZYPISANIE KLAWISZY            |\n";
	std::cout << "==============================================\n";
	std::cout << "|  0  | Obserwacja slonca                    |\n";
	std::cout << "| 1-8 | Obserwacja planet                    |\n";
	std::cout << "==============================================\n";
	std::cout << "|             PREDKOSC  SYMULACJI            |\n";
	std::cout << "==============================================\n";
	std::cout << "|  y  | + 1h/s                               |\n";
	std::cout << "|  h  | - 1h/s                               |\n";
	std::cout << "|  u  | + 1 dni/s                            |\n";
	std::cout << "|  j  | - 1 dni/s                            |\n";
	std::cout << "|  i  | + 1 msc/s                            |\n";
	std::cout << "|  k  | - 1 msc/s                            |\n";
	std::cout << "|  o  | + 1 rok/s                            |\n";
	std::cout << "|  l  | - 1 rok/s                            |\n";
	std::cout << "==============================================\n";

	init();

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(handleKeypress);
	glutMouseFunc(handleMouseClick);
	glutMotionFunc(handleMouseMotion);
	glutIdleFunc(display);
	glutTimerFunc(16, timer, 0);

	glutMainLoop();

	return 0;
}