#include <QHHeadersWin32.h>
#include <HDU/hduMath.h>
#include <HDU/hduMatrix.h>

class DataTransportClass // This class passes data into the ServoLoop thread
{
public:
	TriMesh* Model;				// Trimesh pointer to hold Skull
	Sphere* cursorSphere;		// Sphere pointer for haptic interface point
	Cylinder* forceArrow;		// To show magnitude and direction of force
	Cone* forceArrowTip;		// Tip that points in the force direction
	Cursor* deviceCursor;		// Pointer to hold the cursor data
	Text* descriptionText;
};

// Radius when the inverse square low changes to a spring force law
double chargeRadius = 3;

// This matrix contains the World Space to DeviceSpace Transformation
hduMatrix WorldToDevice;

// This variable contains the force vector
hduVector3Dd forceVec;

// QuickHaptics Graphics callback routine
void GraphicsCallback(void);

//Functions that define HL Custom Force servo loop callback
void HLCALLBACK computeForceCB(HDdouble force[3], HLcache *cache, void *userdata);
void HLCALLBACK startEffectCB(HLcache *cache, void *userdata);
void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata);

// Compute inverse squre forces between the Skull and the particle
hduVector3Dd forceField(hduVector3Dd Pos1, hduVector3Dd Pos2, 
										HDdouble Multiplier, HLdouble Radius);

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
											LPSTR lpCmdLine, int nCmdShow)
{
	QHWin32* DisplayObject = new QHWin32;
	DeviceSpace* OmniSpace = new DeviceSpace;
	DisplayObject->setName("Coulomb Field Demo");
	DisplayObject->tell(OmniSpace);

	// Initialize an object to pass data into the servoloop callback
	DataTransportClass dataObject;

	// Sphere for the haptic interface point
	dataObject.cursorSphere = new Sphere(chargeRadius, 15);
	dataObject.cursorSphere->setName("cursorSphere");
	dataObject.cursorSphere->setShapeColor(0.8,0.2,0.2);

	// Make the Sphere haptically invisible or the proxy will keep colliding
	// with the sphere
	dataObject.cursorSphere->setHapticVisibility(false);
	DisplayObject->tell(dataObject.cursorSphere);

	// Cylinder for the force arrow
	dataObject.forceArrow = new Cylinder(chargeRadius/4,1,15);
	dataObject.forceArrow->setSahpeColor(0.2,0.7,0.2);
	dataObject.forceArrow->setHapticVisibility(false);
	dataObject.forceArrow->setName("forceArrow");
	DisplayObject->tell(dataObject.forceArrow);

	// Cone for the force arrow tip
	dataObject.forceArrowTip = new Cone(2,4,15);
	dataObject.forceArrowTip->setShapeColor(1.0,0.0,0.0);
	dataObject.forceArrowTip->setHapticVisibility(false);
	dataObject.forceArrowTip->setName("forceArrowTip");
	DisplayObject->tell(dataObject.forceArrowTip);

	// Load a Skull Model for the attraction force particle
	dataObject.Model = new TriMesh("models/skull.obj");
	dataObject.Model->setName("Skull");
	dataObject.Model->setHapticVisibility(false);
	dataObject.Model->setShapeColor(0.35,0.15,0.75);

	// Make the skull smaller, about the same size as the sphere
	dataObject.Model->setScale(.2);
	DisplayObject->tell(dataObject.Model);

	// Create a new cursor and make it invisible so that the Red Sphere can be 
	// drawn in its place
	dataObject.deviceCursor = new Cursor;
	dataObject.deviceCursor->setName("devCursor");
	dataObject.deviceCursor->setCursorGraphicallyVisible(false);
	DisplayObject->tell(dataObject.deviceCursor);

	// Text description of this example
	dataObject.descriptionText = new Text(20.0,
		"This example demonstrates Coulomb Forces between two dynamic charges",
		0.1,0.9);

	dataObject.descriptionText->setShapeColor(0.7,0.0,0.4);
	DisplayObject->tell(dataObject.descriptionText);

	// Setup the QuickHaptics graphics callback and the HL Custom Force callback
	DisplayObject->preDrawCallback(GraphicsCallback);

	OmniSpace->startServoLoopCallback(startEffectCB, computeForceCB, 
		stopEffectCB, &dataObject);

	// Change the default camera, first set the Default Camera,
	// then read back the fov, eye point etc.
	DisplayObject->setDefaultCamera();
	float fov, nearplane, farplane;
	hduVector3Dd eyepoint, lookat, up;
	DisplayObject->getCamera(&fov, &nearplane, &farplane, &eyepoint, &lookat, &up);

	eyepoint[2] += 100;		// pull back by 100
	nearplane += 80;		// recenter the haptic workspace (adjust by 20)
	farplane += 80;
	DisplayObject->setCamera(fov+15, nearplane, farplane, eyepoint, lookat, up);

	qhStart();
}

// The QuickHaptics Graphics Callback runs in the application "client thread"
// (qhStart) and sets the transformations for the Red Sphere and Green Line of
// the Cursor. Also, this callback sets the WorldToDevice matrix for later use
// in the HL Custom Force callback which runs in the HD servo loop.
void GraphicsCallback(void)
{
	// Get Pointers to all the QuickHaptics data structures, return if any
	// are missing
	QHWin32* localDisplayObject = QHWin32::searchWindow("Coulomb Field Demo");
	Cursor* localDeviceCursor = Cursor::searchCursor("devCursor");
	Cylinder* localForceArrow = Cylinder::searchCylinder("forceArrow");
	Cone* localForceArrowTip = Cone:searchCone("forceArrowTip");
	Sphere* localCursorSphere = Sphere::searchSphere("cursorSphere");

	if (localDisplayObject == NULL || localDeviceCursor == NULL ||
		localForceArrow == NULL || localCursorSphere == NULL)
		return;

	hduMatrix CylinderTransform;
	hduVector3Dd localCursorPosition;
	hduVector3Dd DirectionVecX;
	hduVector3Dd PointOnPlane;
	hduVector3Dd DirectionVecY;
	hduVector3Dd DirectionVecZ;

	// Compute the world to device transform
	WorldToDevice = localDisplayObject->getWorldToDeviceTransform();

	// Set transform for Red Sphere based on the cursor position
	// in World Space
	localCursorPosition = localDeviceCursor->getPosition();
	hduVector3Dd localCursorSpherePos = localCursorSphere->getTranslation();

	localCursorSphere->setTranslation(-lcalCursorSpherePos);	// reset position
	localCursorSphere->setTranslation(localCursorPosition);

	//////////////////////////////////////////////////////////////////////////////////
	// Calculate transform of the green cylinder to point along the force direction //
	//////////////////////////////////////////////////////////////////////////////////
	hduMatrix DeviceToWorld = WorldToDevice.getInverse();
	HDdouble ForceMagnitude = forceVec.magnitude();
	DeviceToWorld[3][0] = 0.0;
	DeviceToWorld[3][1] = 0.0;
	DeviceToWorld[3][2] = 0.0;
	DirectionVecX = forceVec * DeviceToWorld;
	DirectionVecX.normalize();
	PointOnPlane.set(0.0,0.0,(DirectionVecX[0]*localCursorPosition[0] + DirectionVecX[1]*localCursorPosition[1] + DirectionVecX[2]*localCursorPosition[2])/DirectionVecX[2]);
	
	DirectionVecY = PointOnPlane - localCursorPosition;
	DirectionVecY.normalize();

	DirectionVecZ = -DirectionVecY.crossProduct(DirectionVecX);

	CylinderTransform[0][0] = DirectionVecZ[0];
	CylinderTransform[0][1] = DirectionVecZ[1];
	CylinderTransform[0][2] = DirectionVecZ[2];
	CylinderTransform[0][3] = 0.0;

	CylinderTransform[1][0] = DirectionVecX[0];
	CylinderTransform[1][1] = DirectionVecX[1];
	CylinderTransform[1][2] = DirectionVecX[2];
	CylinderTransform[1][3] = 0.0;

	CylinderTransform[2][0] = DirectionVecY[0];
	CylinderTransform[2][1] = DirectionVecY[1];
	CylinderTransform[2][2] = DirectionVecY[2];
	CylinderTransform[2][3] = 0.0;

	CylinderTransform[3][0] = 0.0;
	CylinderTransform[3][1] = 0.0;
	CylinderTransform[3][2] = 0.0;
	CylinderTransform[3][3] = 1.0;

	CylinderTransform = CylinderTransform * 
		hduMatrix::createTranslation(localCursorPosition[0], localCursorPosition[1], 
			localCursorPosition[2]);

	localForceArrow->update(chargeRadius/4, ForceMagnitude*50, 15);
	localForceArrow->setTransform(CylinderTransform);

	hduMatrix ConeTransform = CylinderTransform *
		hduMatrix::createTranslation(DirectionVecX[0] * ForceMagnitude * 50,
			DirectionVecX[1] * ForceMagnitude*50,
			DirectionVecX[2] * ForceMagnitude*50);

	localForceArrowTip->setTransform(ConeTransform);

}