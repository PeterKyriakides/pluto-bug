/*
  PlutoBUG_OS Header File   |  Version: 1.0.0
  by plutoLABS              |           01/09/2023
 --------------------------------------------------
*/

//**************************************************************************
// Robotic [PS3] Controller System Setup
//**************************************************************************
// Define controller initialisation flag
bool controller_isInit = false;

// Define controller battery status
int controllerBattery = 0;

// Define starting move of standing
bool move_isInit = false;

// Define joystick trigger variables
int stickSenseMAX = 100;
int stickSenseMIN = 5;

//**************************************************************************
// Posture Management System Setup:
//**************************************************************************
// Define posture check flag
bool isStanding = false;
bool isWalking = false;

// Define gait posistion check flag
bool walkForward = false;
bool walkBack = false;
bool walkHalt = false;

bool pivotRight = false;
bool pivotLeft = false;

// Define alternate gait pattern flag
bool altLeft = false;
bool altRight = false;

bool altWalkFwd = false;
bool altWalkBck = false;

//**************************************************************************
// Joint Management System Setup:
//**************************************************************************
// Servo Joint Speed Setting
//--------------------------------
unsigned long MAX_SPEED = 2.0; // Min no. of ms per deg || i.e. lower = faster!

// Servo Joint Channel Definition:
//---------------------------------
// - Inner Servos
#define servo_A1 4
#define servo_B1 18
#define servo_C1 13
#define servo_D1 14

// - Outer Servos
#define servo_A2 2
#define servo_B2 23
#define servo_C2 12
#define servo_D2 27

// Servo Joint Setup:
//--------------------
Servo joint_A1, joint_B1, joint_C1, joint_D1;
Servo joint_A2, joint_B2, joint_C2, joint_D2;

// Define Joint(s) Check Flags:
//---------------------
bool joints_isInit = false;

// Joint Zero-Angle Definition:
//---------------------------------
// - Inner Servos
#define zero_A1 0
#define zero_B1 180 // Flipped
#define zero_C1 180 // Flipped
#define zero_D1 0

// - Outer Servos
#define zero_A2 90
#define zero_B2 90
#define zero_C2 90
#define zero_D2 90

// Predefine Limb Servo Angle Structures
//--------------------------------
float currentAngle[4][2] = {{0, 90},   // Leg_A  = {Joint_A1, Joint_A2}
                            {0, 90},   // Leg_B  = {Joint_B1, Joint_B2}
                            {0, 90},   // Leg_C  = {Joint_C1, Joint_C2}
                            {0, 90}};  // Leg_D  = {Joint_D1, Joint_D2}
                            
// ************************ STANDING / SQUATTING SEQUENCE ************************
float frontStand[4][2] = {{0, 0},
                          {0, 0},
                          {0, 0},
                          {0, 0}};
                          
float crossStand[4][2] = {{45, 0},
                          {45, 0},
                          {45, 0},
                          {45, 0}};

float sideStand[4][2] = {{90, 0},
                         {90, 0},
                         {90, 0},
                         {90, 0}};

float frontSquat[4][2] = {{0, 35},
                          {0, 35},
                          {0, 35},
                          {0, 35}};
                          
float crossSquat[4][2] = {{45, 35},
                          {45, 35},
                          {45, 35},
                          {45, 35}};

float sideSquat[4][2] = {{90, 35},
                         {90, 35},
                         {90, 35},
                         {90, 35}};

float layAngle[4][2] = {{0, 75},
                        {0, 75},
                        {0, 75},
                        {0, 75}};
                         
// ************************ STARTUP SEQUENCE ************************
float startStand_A[4][2] = {{0, 25},
                            {0, 0},
                            {0, 0},
                            {0, 25}};
                            
float startStand_B[4][2] = {{45, 0},
                            {0, 0},
                            {0, 0},
                            {45, 0}};

float startStand_C[4][2] = {{45, 0},
                            {0, 25},
                            {0, 25},
                            {45, 0}};
                            
float startStand_D[4][2] = {{45, 0},
                            {45, 0},
                            {45, 0},
                            {45, 0}};
                           
// ************************ BOWING DOWN SEQUENCE ************************
float bowUp[4][2] = {{0, 0},
                     {45, 0},
                     {0, 0},
                     {45, 0}};

float bowDown[4][2] = {{0, 90},
                       {45, 0},
                       {0, 90},
                       {45, 0}};

// ************************ WAVING SEQUENCE ************************
float waveUp[4][2] = {{45, 160},
                      {65, 0},
                      {25, 0},
                      {45, 0}};

float waveLeft[4][2] = {{80, 160},
                        {65, 0},
                        {25, 0},
                        {45, 0}};
  
float waveRight[4][2] = {{10, 160},
                         {65, 0},
                         {25, 0},
                         {45, 0}};
                         
// ************************ PUSH-UP SEQUENCE ************************
float pushUp_UP[4][2] = {{80, 15},  // FL
                         {10, 50},  // BL
                         {80, 15},  // FR
                         {10, 50}}; // BR

float pushUp_DOWN[4][2] = {{80, 60},  // FL
                           {10, 50},  // BL
                           {80, 60},  // FR
                           {10, 50}}; // BR

// ************************ WALK FORWARD SEQUENCE ************************
float forwardWalk_A[4][2] = {{45, 25},   // FL
                             {65, 0},   // BL
                             {25, 0},   // FR
                             {45, 25}};  // BR

float forwardWalk_B[4][2] = {{25, 25},   // FL
                             {25, 0},   // BL
                             {65, 0},   // FR
                             {65, 25}};  // BR

float forwardWalk_C[4][2] = {{25, 0},   // FL
                             {45, 25},   // BL
                             {45, 25},   // FR
                             {65, 0}};  // BR

float forwardWalk_D[4][2] = {{65, 0},   // FL
                             {65, 25},   // BL
                             {25, 25},   // FR
                             {25, 0}};  // BR

// ************************ WALK BACK SEQUENCE ************************
float backWalk_A[4][2] = {{45, 25},   // FL
                          {25, 0},   // BL
                          {65, 0},   // FR
                          {45, 25}};  // BR

float backWalk_B[4][2] = {{65, 25},   // FL
                          {65, 0},   // BL
                          {25, 0},   // FR
                          {25, 25}};  // BR

float backWalk_C[4][2] = {{65, 0},   // FL
                          {45, 25},   // BL
                          {45, 25},   // FR
                          {25, 0}};  // BR

float backWalk_D[4][2] = {{25, 0},   // FL
                          {25, 25},   // BL
                          {65, 25},   // FR
                          {65, 0}};  // BR

// ************************ PIVOT RIGHT(1) SEQUENCE ************************
float rightPivot_A1[4][2] = {{45, 25},   // FL
                             {0, 10},   // BL
                             {0, 10},   // FR
                             {45, 25}};  // BR

float rightPivot_B1[4][2] = {{0, 25},   // FL
                             {0, 10},   // BL
                             {0, 10},   // FR
                             {0, 25}};  // BR

float rightPivot_C1[4][2] = {{0, 0},   // FL
                             {0, 10},   // BL
                             {0, 10},   // FR
                             {0, 0}};  // BR

float rightPivot_D1[4][2] = {{90, 0},   // FL
                             {0, 10},   // BL
                             {0, 10},   // FR
                             {90, 0}};  // BR

// ************************ PIVOT RIGHT(2) SEQUENCE ************************
float rightPivot_A2[4][2] = {{90, 10},   // FL
                             {45, 25},   // BL
                             {45, 25},   // FR
                             {90, 10}};  // BR

float rightPivot_B2[4][2] = {{90, 10},   // FL
                             {90, 25},   // BL
                             {90, 25},   // FR
                             {90, 10}};  // BR

float rightPivot_C2[4][2] = {{90, 10},   // FL
                             {90, 0},   // BL
                             {90, 0},   // FR
                             {90, 10}};  // BR

float rightPivot_D2[4][2] = {{90, 10},   // FL
                             {0, 0},   // BL
                             {0, 0},   // FR
                             {90, 10}};  // BR

// ************************ PIVOT LEFT(1) SEQUENCE ************************
float leftPivot_A1[4][2] = {{45, 25},   // FL
                            {90, 10},   // BL
                            {90, 10},   // FR
                            {45, 25}};  // BR

float leftPivot_B1[4][2] = {{90, 25},   // FL
                            {90, 10},   // BL
                            {90, 10},   // FR
                            {90, 25}};  // BR

float leftPivot_C1[4][2] = {{90, 0},   // FL
                            {90, 10},   // BL
                            {90, 10},   // FR
                            {90, 0}};  // BR

float leftPivot_D1[4][2] = {{0, 0},   // FL
                            {90, 10},   // BL
                            {90, 10},   // FR
                            {0, 0}};  // BR

// ************************ PIVOT LEFT(2) SEQUENCE ************************
float leftPivot_A2[4][2] = {{0, 10},   // FL
                            {45, 25},   // BL
                            {45, 25},   // FR
                            {0, 10}};  // BR

float leftPivot_B2[4][2] = {{0, 10},   // FL
                            {0, 25},   // BL
                            {0, 25},   // FR
                            {0, 10}};  // BR

float leftPivot_C2[4][2] = {{0, 10},   // FL
                            {0, 0},   // BL
                            {0, 0},   // FR
                            {0, 10}};  // BR

float leftPivot_D2[4][2] = {{0, 10},   // FL
                            {90, 0},   // BL
                            {90, 0},   // FR
                            {0, 10}};  // BR
