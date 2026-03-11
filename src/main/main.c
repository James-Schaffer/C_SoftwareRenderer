#include <stdio.h>
#include <stdlib.h>
#include <SDL3/SDL.h>

#include "mesh.h"
#include "vector.h"
#include "window.h"

#define SDL_MAIN_HANDLED

// WRITEN BY JAMES SCHAFFER 2026
//
// SIMPLE SOFTWARE RENDERER, C & SDL
//
// Note world axis :
//
// x -left/right+
// y -down/up+
// z -back/front+

// ========== DEFINITIONS ==========
#define PI	3.1415926535897932384
#define PI_2	1.5707963267948966192

#define DEG2RAD(x) ((x) * (PI / 180.0))

#define SDL_WINDOW_WIDTH 1920U
#define SDL_WINDOW_HEIGHT 1080U

#define CAM_CLIP_MIN 0.5

#define MAX_VERTEX 10000U
#define MAX_FACES  10000U

// ========== STRUCTS ==========

typedef struct  {
	v3 position;
	v3 rotation;
	v3 defNormal;
	v3 defUp;
	double fov;
} CamState;
typedef struct {
	v3 position;
	v3 planePosition;
	v3 normalV;
	v3 upV;
	v3 rightV;
	double fov_scale;
} CamProjectionInfo;

// ========== OTHER VARS ==========

bool gameRunning = true;

// ========== INPUT BOOLS ==========
// Rotation
bool spaceDown = false;
bool spinToggle = false;

bool xDown = false;
bool yDown = false;
bool zDown = false;

// Scale
bool jDown = false;
bool kDown = false;

// Cam move
bool wDown = false;
bool aDown = false;
bool sDown = false;
bool dDown = false;

bool eDown = false;
bool qDown = false;

// ========== CAMERA TRANSFORM ==========

CamState cam = {{0, -2, 0}, {0,0,0}, {0,1,0}, {0,0,1}, PI_2};

// Other

v3 sun = {0, 1, -1};
Transform meshTrans = { {0,0,0}, {0,0,0}, {1,1,1}};

// ========== SETUP CAM PROJECTION VARS FOR EACH FRAME ==========

CamProjectionInfo getCamProjectionInfo(const CamState* camera) {
	CamProjectionInfo ret;

	ret.position = camera->position;

	Transform camTransform = {
		{0,0,0},
		camera->rotation,
		{1,1,1}
	};

	ret.normalV = normalize(transformV3(&camera->defNormal,&camTransform));
	ret.upV = normalize(transformV3(&camera->defUp,&camTransform));

	// Projection plane
	const v3 scaledNormal = v3Scale(ret.normalV, CAM_CLIP_MIN);
	const v3 planePoint = v3Add(camera->position, scaledNormal);

	ret.planePosition = planePoint;

	// Right vector
	ret.rightV = normalize(crossProduct(ret.upV, ret.normalV));

	// fov scale
	ret.fov_scale = SDL_WINDOW_WIDTH / (2 * tan(camera->fov / 2));

	return ret;
}

// ========== PROJECT A POINT IN 3D SPACE TO A 2D POSITION ON SCREEN ==========

int project3DtoScreen(const v3 point, const CamProjectionInfo* camState, v2* outV) {
	// Ray
	const v3 ray = normalize(v3Sub(point, camState->position));

	// given t = (a-p0).n / v.n (a=planeCenter , p0=camPos, n=normal, v=rayVector(normalized))

	const double vn = dotProduct(ray, camState->normalV);
	if (fabs(vn) < 1e-6) return 0; // parallel to plane (fabs = float absolute value)

	const double t = dotProduct(v3Sub(camState->planePosition, camState->position), camState->normalV) / vn;
	if (t <= 0.0) return 0; // Behind camera

	// Find intersection point
	v3 hit = v3Add(camState->position, v3Scale(ray, t));

	// Find local intersection point (relative to plane center)
	const v3 hit_planeSpace = v3Sub(hit, camState->planePosition);

	// Find x y coords on plane for intersection (0,0 center and + axis is up and right)
	double x = dotProduct(hit_planeSpace, camState->rightV);
	double y = dotProduct(hit_planeSpace, camState->upV);

	x *= camState->fov_scale;
	y *= camState->fov_scale;

	// the dot product gives x and y where 0 is center of screen so :
	// re-map 0,0 to top left and + axis to right down
	outV->x = x + SDL_WINDOW_WIDTH / 2;
	outV->y = y + SDL_WINDOW_HEIGHT / 2;

	return 1;
}

// ========== Projects an array of points to the screen ==========

void projectPoints3DtoScreen(const v3* v, v2* projected, const int n, const CamState* camState) {
	const CamProjectionInfo camInfo = getCamProjectionInfo(camState);

	for (int i=0; i<n; ++i) {
		v2 projectionPoint;

		int out = project3DtoScreen(v[i], &camInfo, &projectionPoint);

		if (out==1) projected[i] = projectionPoint;
		else {
			projected[i].x = 0;
			projected[i].y = 0;
		};
	}
}

// ===== UPDATE LOOP =====

void update(double delta) {
	if (spinToggle) {
		meshTrans.rotation.x += PI * 0.4 * delta;
		meshTrans.rotation.y += PI * 0.3 * delta;
		meshTrans.rotation.z += PI * 0.5 * delta;
	}

	if (xDown) {
		meshTrans.rotation.x += PI * 0.4 * delta;
	}
	if (yDown) {
		meshTrans.rotation.y += PI * 0.4 * delta;
	}
	if (zDown) {
		meshTrans.rotation.z += PI * 0.4 * delta;
	}

	if (jDown) {
		meshTrans.scale.x += 0.1 * delta;
		meshTrans.scale.y += 0.1 * delta;
		meshTrans.scale.z += 0.1 * delta;
	}
	if (kDown) {
		meshTrans.scale.x -= 0.1 * delta;
		meshTrans.scale.y -= 0.1 * delta;
		meshTrans.scale.z -= 0.1 * delta;
	}

	// x,y plane
	v2 moveDir = {0,0};

	if (wDown) {
		moveDir.y += 1;
	}
	if (sDown) {
		moveDir.y -= 1;
	}
	if (aDown) {
		moveDir.x += 1;
	}
	if (dDown) {
		moveDir.x -= 1;
	}


	moveDir = normalizev2(moveDir);

	double ct = cos(cam.rotation.z);
	double st = sin(cam.rotation.z);

	moveDir = (v2){
		moveDir.x*ct - moveDir.y*st,
		moveDir.x*st + moveDir.y*ct
	};

	cam.position.x += moveDir.x * 2 * delta;
	cam.position.y += moveDir.y * 2 * delta;

	// Up down
	if (qDown) {
		cam.position.z -= 2 * delta;
	}
	if (eDown) {
		cam.position.z += 2 * delta;
	}
}

// ===== RENDER FRAME =====
void render(SDL_Renderer* renderer, Mesh mesh) {
	// Clear screen
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	SDL_RenderClear(renderer);

	SDL_FColor colf = {
		0, 0, 0, 1
	};

	v3 points[3];
	v2 projectedPoints[3];
	SDL_Vertex verts[3];

	for (int i=0; i<mesh.faceCount; ++i) {
		//v3 normal = mesh.normals[mesh.faces[i].n0];

		v3 viewDir = normalize(v3Sub(mesh.vertices[mesh.faces[i].v0], cam.position));

		if (dotProduct(mesh.normals[mesh.faces[i].n0], viewDir) > 0) {
			continue; // Skip if facing away from cam
		}

		// points[0] = transformV3(&mesh.vertices[mesh.faces[i].v0], &meshTrans);
		// points[1] = transformV3(&mesh.vertices[mesh.faces[i].v1], &meshTrans);
		// points[2] = transformV3(&mesh.vertices[mesh.faces[i].v2], &meshTrans);

		points[0] = mesh.vertices[mesh.faces[i].v0];
		points[1] = mesh.vertices[mesh.faces[i].v1];
		points[2] = mesh.vertices[mesh.faces[i].v2];

		// colf.r = (( (unsigned int)((i%255)*23.324234543) )%255)/255.0;
		// colf.g = (( (unsigned int)((i%255)*14.932543) )%255)/255.0;
		// colf.b = (( (unsigned int)((i%255)*3.24234) )%255)/255.0;

		double intensity = dotProduct(mesh.normals[mesh.faces[i].n0], viewDir);

		intensity *= -1;

		if (intensity < 0.0)
			intensity = 0.0;
		if (intensity > 1.0)
			intensity = 1.0;

		colf.r = intensity;
		colf.g = intensity;
		colf.b = intensity;

		projectPoints3DtoScreen(points, projectedPoints, 3, &cam);

		// Triangle 1 (0,1,2)
		verts[0] = (SDL_Vertex){ {projectedPoints[0].x, projectedPoints[0].y}, colf };
		verts[1] = (SDL_Vertex){ {projectedPoints[1].x, projectedPoints[1].y}, colf };
		verts[2] = (SDL_Vertex){ {projectedPoints[2].x, projectedPoints[2].y}, colf };

		SDL_RenderGeometry(renderer, NULL, verts, 3, NULL, 0);
	}
	SDL_RenderPresent(renderer);
}


// HANDLE INPUTS

void quitGame() {
	printf("Quitting...\n");
	gameRunning = false;
}

void manageKeyDownEvent(const SDL_KeyboardEvent *e) {
	switch (e->key) {
		case SDLK_ESCAPE:
			quitGame();
			break;

		case SDLK_SPACE:
			if (spaceDown) break;
			spinToggle = !spinToggle;
			spaceDown=true;
			break;

		case SDLK_X:
			if (xDown) break;
			xDown=true;
			break;
		case SDLK_Y:
			if (yDown) break;
			yDown=true;
			break;
		case SDLK_Z:
			if (zDown) break;
			zDown=true;
			break;

		case SDLK_W:
			if (wDown) break;
			wDown=true;
			break;
		case SDLK_A:
			if (aDown) break;
			aDown=true;
			break;
		case SDLK_S:
			if (sDown) break;
			sDown=true;
			break;
		case SDLK_D:
			if (dDown) break;
			dDown=true;
			break;

		case SDLK_E:
			if (eDown) break;
			eDown=true;
			break;
		case SDLK_Q:
			if (qDown) break;
			qDown=true;
			break;

		case SDLK_J:
			if (jDown) break;
			jDown=true;
			break;
		case SDLK_K:
			if (kDown) break;
			kDown=true;
			break;
		default:
			//printf("KeyDown\n");
			break;
	}
}

void manageKeyUpEvent(const SDL_KeyboardEvent *e) {
	switch (e->key) {
		case SDLK_SPACE:
			if (!spaceDown) break;
			spaceDown=false;
			break;

		case SDLK_X:
			if (!xDown) break;
			xDown=false;
			break;
		case SDLK_Y:
			if (!yDown) break;
			yDown=false;
			break;
		case SDLK_Z:
			if (!zDown) break;
			zDown=false;
			break;

		case SDLK_W:
			if (!wDown) break;
			wDown=false;
			break;
		case SDLK_A:
			if (!aDown) break;
			aDown=false;
			break;
		case SDLK_S:
			if (!sDown) break;
			sDown=false;
			break;
		case SDLK_D:
			if (!dDown) break;
			dDown=false;
			break;

		case SDLK_E:
			if (!eDown) break;
			eDown=false;
			break;
		case SDLK_Q:
			if (!qDown) break;
			qDown=false;
			break;

		case SDLK_J:
			if (!jDown) break;
			jDown=false;
			break;
		case SDLK_K:
			if (!kDown) break;
			kDown=false;
			break;
		default:
			//printf("KeyUp\n");
			break;
	}
}

void manageMouseMotion(const SDL_MouseMotionEvent *e) {
	double sense = 0.005;

	double deltaX = e->xrel * sense;
	double deltaY = e->yrel * sense;

	cam.rotation.z += deltaX;
	cam.rotation.x += deltaY;

	if (cam.rotation.x > PI_2) {
		cam.rotation.x = PI_2;
	} else if (cam.rotation.x < -PI_2) {
		cam.rotation.x = -PI_2;
	}
}

void manageMouseScroll(SDL_MouseWheelEvent * wheel) {
	cam.fov += wheel->y * -0.1;
}


int main(void) {
	printf("Hello, World!\n");

	Window window = createWindow(SDL_WINDOW_WIDTH, SDL_WINDOW_HEIGHT);

	if (!SDL_Init(SDL_INIT_VIDEO)) {
		SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
		return SDL_APP_FAILURE;
	}

	initWindow(window);
	SDL_Renderer* renderer = SDL_CreateRenderer(window->window, NULL);

	if (!renderer) {
		SDL_Log("Failed to create renderer: %s", SDL_GetError());
	}

	//Lock mouse to screen center
	SDL_SetWindowRelativeMouseMode(window->window, true);

	Uint64 now = SDL_GetPerformanceCounter();
	Uint64 last = 0;
	double deltaTime = 0.0;

	long double timeAccum = 0.0;
	Uint64 frames = 0;

	SDL_Event e;

	int meshCount;
	Mesh* meshes = loadMeshFromOBJ("cat.obj", &meshCount);

	while (gameRunning) {
		// Update deltaTime
		last = now;
		now = SDL_GetPerformanceCounter();
		deltaTime = (double)(now - last) / (double)SDL_GetPerformanceFrequency();

		// FPS
		timeAccum += deltaTime;
		frames++;
		if (timeAccum > 1) {
			timeAccum -= 1;
			printf("%ifps\n", frames);
			frames = 0;
		}

		// Event handler
		while (SDL_PollEvent(&e)) {
			switch (e.type) {
				case SDL_EVENT_QUIT:
					quitGame();
					break;
				case SDL_EVENT_KEY_DOWN:
					manageKeyDownEvent(&e.key);
					break;
				case SDL_EVENT_KEY_UP:
					manageKeyUpEvent(&e.key);
					break;
				case SDL_EVENT_MOUSE_MOTION:
					manageMouseMotion(&e.motion);
					break;
				case SDL_EVENT_MOUSE_WHEEL:
					manageMouseScroll(&e.wheel);
					break;
				default:
					//printf("Event\n");
					break;
			}
		}

		update(deltaTime);
		render(renderer, meshes[0]);

	}

	// Cleanup
	freeMesh(&meshes[0]);

	SDL_DestroyRenderer(renderer);
	destroyWindow(window);
	SDL_Quit();

	return 0;
}
