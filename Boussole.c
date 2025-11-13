#pragma config(Sensor, S1,     touchSensor,    sensorEV3_Touch)
#pragma config(Sensor, S4,     gyroSensor,     sensorEV3_Gyro)
#pragma config(Motor,  motorA,          Uno,           tmotorEV3_Medium, PIDControl, encoder)

// ============================================================================
// CONSTANTES ET VARIABLES GLOBALES
// ============================================================================

const float P = 1;    // Terme proportionnel :
// Il détermine la force de correction proportionnelle à l'écart instantané entre la consigne et la position actuelle.
// Plus P est grand, plus le moteur réagit vite pour réduire l'erreur, mais trop grand peut provoquer des oscillations.

const float I = 0.01;   // Terme intégral :
// Il corrige les erreurs persistantes dans le temps (décalages ou biais).
// Il accumule les erreurs passées pour éliminer un écart constant.
// Ici, une petite valeur pour éviter que le système devienne instable ou ait des "à-coups".

const float D = -0.45;   // Terme dérivé :
// Il agit comme un amortisseur basé sur la vitesse du moteur (variation de la position).
// Ce terme freine les mouvements brusques, réduisant les oscillations autour de la consigne.
// La valeur négative indique qu?il agit en sens inverse de la vitesse angulaire pour stabiliser.
int consigne = 0;
bool terminate = false;
TSemaphore mutexConsigne;



// ============================================================================
// TÂCHE launchMotorSpeed
// ============================================================================
void launchMotorSpeed(int speed)
{
	int power;
	const int maxPower = 100;     // puissance max
	const int maxSpeed = 850;     // vitesse max en RPM
	const int minPower = 10;      // puissance min
	const int slope = maxSpeed / maxPower;  // pente = RPM par unit? de puissance

	if (abs(speed) > maxSpeed)
	{
		speed = (speed > 0) ? maxSpeed : -maxSpeed;
	}

	power = speed / slope;


	if (abs(power) < minPower && speed != 0)
	{
		power = (power > 0) ? minPower : -minPower;
	}

	motor[motorA] = power;
}

// ============================================================================
// FONCTION launchMotorSpeed2
// ============================================================================
void launchMotorSpeed2(int speed)
{
	int power;
	const int minPower = 10;
	const int maxPower = 100;
	const int maxSpeed2 = 500;

	if (abs(speed) > maxSpeed2)
		speed = (speed > 0) ? maxSpeed2 : -maxSpeed2;

	int gyroSpeed = getGyroRate(S4);
	int error = speed - gyroSpeed;
	float Kp = 0.3;
	power = (int)(Kp * error);

	if (power > maxPower) power = maxPower;
	else if (power < -maxPower) power = -maxPower;

	if (abs(power) < minPower && speed != 0)
		power = (power > 0) ? minPower : -minPower;
	else if (speed == 0)
		power = 0;

	motor[motorA] = power;
}

// ============================================================================
// FONCTION initialize
// ============================================================================
void initialize()
{
	bool done = false;

	while(!done)
	{
		eraseDisplay();
		displayTextLine(0, "=== INITIALISATION ===");
		displayTextLine(2, "Gauche : sens antihoraire");
		displayTextLine(3, "Droite : sens horaire");
		displayTextLine(5, "Centre : valider");

		if(getButtonPress(buttonLeft))
			motor[motorA] = -20;
		else if(getButtonPress(buttonRight))
			motor[motorA] = 20;
		else if(getButtonPress(buttonEnter))
		{
			while(getButtonPress(buttonEnter)) wait1Msec(10);
			done = true;
		}
		else
			motor[motorA] = 0;

		wait1Msec(50);
	}

	motor[motorA] = 0;
	resetGyro(S4);
	wait1Msec(500);

	eraseDisplay();
	displayTextLine(3, "Position initiale OK");
	wait1Msec(1000);
}



// ============================================================================
// TÂCHE keepHeadingPID
// ============================================================================
task keepHeadingPID()
{



#define INTEGRAL_SIZE 10
	int erreurs[INTEGRAL_SIZE];
	int indexErreur = 0;
	int sommeErreurs = 0;

	while(1)
	{
		int capActuel = getMotorEncoder(motorA);
		int vitesseAngulaire = getMotorRpm(motorA);

		int erreur;
		semaphoreLock(mutexConsigne);
		erreur = consigne - capActuel;
		semaphoreUnlock(mutexConsigne);

		sommeErreurs -= erreurs[indexErreur];
		erreurs[indexErreur] = erreur;
		sommeErreurs += erreur;

		indexErreur = (indexErreur + 1) % INTEGRAL_SIZE;

		if (abs(erreur) < 4 )
		{
			launchMotorSpeed(0);
		}
		else
		{
			int vitesse = (int)(P * erreur + I * sommeErreurs + D * vitesseAngulaire);
			launchMotorSpeed(vitesse);
		}

		wait1Msec(30);
	}
}

// ============================================================================
// TÂCHE keepHeadingPID2
// ============================================================================
task keepHeadingPID2()
{
	const int dt = 30;
	int erreurs[10];
	int indexErreur = 0;
	int sommeErreurs = 0;

	for(int i = 0; i < 10; i++) erreurs[i] = 0;

	int lastGyroAngle = SensorValue[S4];
	int lastTime = time1[T1];

	while(!terminate)
	{
		int currentTime = time1[T1];
		int gyroAngle = SensorValue[S4];

		int deltaTime = currentTime - lastTime;
		if(deltaTime == 0) deltaTime = 1;


		int vitesseAngulaire = getGyroRate(S4);

		lastGyroAngle = gyroAngle;
		lastTime = currentTime;

		semaphoreLock(mutexConsigne);
		int erreur = consigne - gyroAngle;
		semaphoreUnlock(mutexConsigne);

		sommeErreurs -= erreurs[indexErreur];
		erreurs[indexErreur] = erreur;
		sommeErreurs += erreur;
		indexErreur = (indexErreur + 1) % 10;

		if (abs(erreur) < 4)
			launchMotorSpeed2(0);
		else
		{
			int vitesseCommande = (int)(P * erreur + I * sommeErreurs - D * vitesseAngulaire);
			if(abs(vitesseCommande) > 300)
				vitesseCommande = (vitesseCommande > 0) ? 300 : -300;

			launchMotorSpeed2(vitesseCommande);
		}

		wait1Msec(dt);
	}

	motor[motorA] = 0;
}


// ============================================================================
// TÂCHE watchButtons
// ============================================================================
task watchButtons()
{
    while(!terminate)
    {
        if (getButtonPress(buttonRight))
        {
            semaphoreLock(mutexConsigne);
            consigne += 10;
            semaphoreUnlock(mutexConsigne);
            while(getButtonPress(buttonRight)) {} // anti-rebond
        }
        else if (getButtonPress(buttonLeft))
        {
            semaphoreLock(mutexConsigne);
            consigne -= 10;
            semaphoreUnlock(mutexConsigne);
            while(getButtonPress(buttonLeft)) {}
        }
        else if (getButtonPress(buttonUp))
        {
            semaphoreLock(mutexConsigne);
            consigne += 90;
            semaphoreUnlock(mutexConsigne);
            while(getButtonPress(buttonUp)) {}
        }
        else if (getButtonPress(buttonDown))
        {
            semaphoreLock(mutexConsigne);
            consigne -= 90;
            semaphoreUnlock(mutexConsigne);
            while(getButtonPress(buttonDown)) {}
        }
        else if (getButtonPress(buttonEnter))
        {
            terminate = true;
            stopAllTasks();
        }

        wait1Msec(100);
    }
}


// ============================================================================
// TÂCHE watchButtons2
// ============================================================================
task watchButtons2()
{
	while(!terminate)
	{
		if(getButtonPress(buttonEnter))
		{
			while(getButtonPress(buttonEnter)) wait1Msec(10);
			terminate = true;
			break;
		}
		wait1Msec(50);
	}
}

// ============================================================================
// TÂCHE IHM
// ============================================================================
task IHM()
{

        int currentConsigne;



        eraseDisplay();
        displayString(1, "Boutons:");
        displayString(2, "Droite : +10 deg");
        displayString(3, "Gauche : -10 deg");
        displayString(4, "Haut   : +90 deg");
        displayString(5, "Bas    : -90 deg");
        displayString(6, "Centre : Quitter");
				while(!terminate){

				semaphoreLock(mutexConsigne);
        currentConsigne = consigne;
        semaphoreUnlock(mutexConsigne);

        displayString(8, "Consigne : %d deg", currentConsigne);
        displayString(9, "Position : %d deg", nMotorEncoder[motorA]);
			}
}
// ============================================================================
// TÂCHE IHM2
// ============================================================================
task IHM2()
{
	while(!terminate)
	{
		eraseDisplay();
		displayTextLine(0, "Bouton central = QUITTER");

		semaphoreLock(mutexConsigne);
		int consigneLocale = consigne;
		semaphoreUnlock(mutexConsigne);
		int capActuel = SensorValue[S4];

		displayTextLine(2, "Consigne: %d deg", consigneLocale);
		displayTextLine(3, "Cap: %d deg", capActuel);
		displayTextLine(4, "Erreur: %d deg", consigneLocale - capActuel);
		displayTextLine(5, "Moteur: %d", motor[motorA]);

		wait1Msec(100);
	}

	eraseDisplay();
	displayTextLine(3, "Programme arrete");
	displayTextLine(4, "Retour au menu...");
	wait1Msec(1500);
}

// ============================================================================
// FONCTION boussoleMode
// ============================================================================
void boussoleMode()
{
	terminate = false;

	semaphoreInitialize(mutexConsigne);
	initialize();

	semaphoreLock(mutexConsigne);
	consigne = SensorValue[S4];
	semaphoreUnlock(mutexConsigne);

	eraseDisplay();
	displayTextLine(2, "Mode Boussole");
	displayTextLine(4, "Initialisation...");
	playSound(soundBeepBeep);
	wait1Msec(1000);

	startTask(keepHeadingPID2);
	startTask(watchButtons2);
	startTask(IHM2);

	while(!terminate)
		wait1Msec(100);

	motor[motorA] = 0;
	playSound(soundDownwardTones);
	wait1Msec(1000);
}

// ============================================================================
// FONCTION FixeMode
// ============================================================================

void FixeMode()
{
	terminate = false;

	// Initialisation : permettre à l'utilisateur de positionner le gyroscope
	eraseDisplay();
	displayTextLine(0, "=== Robot Fixe ===");
	displayTextLine(2, "Positionner le gyro");
	displayTextLine(3, "Gauche/Droite = tourner");
	displayTextLine(4, "Centre = valider");

	bool initDone = false;
	while(!initDone)
	{
		if(getButtonPress(buttonLeft))
			motor[motorA] = -20;
		else if(getButtonPress(buttonRight))
			motor[motorA] = 20;
		else if(getButtonPress(buttonEnter))
		{
			while(getButtonPress(buttonEnter)) wait1Msec(10);
			initDone = true;
		}
		else
			motor[motorA] = 0;

		wait1Msec(50);
	}
	motor[motorA] = 0;
	resetGyro(S4);
	wait1Msec(500);

	// Initialisation de la consigne
	semaphoreLock(mutexConsigne);
	consigne = SensorValue[S4];
	semaphoreUnlock(mutexConsigne);

	// Affichage et tâches
	eraseDisplay();
	displayTextLine(2, "Mode Robot Fixe");
	displayTextLine(4, "Bouton central = retour menu");
	playSound(soundBeepBeep);
	wait1Msec(1000);

	startTask(keepHeadingPID);  // Régulation PID
	startTask(IHM);             // Affichage de la consigne et position
	startTask(watchButtons);   // Pour quitter avec le bouton central

	// Arrêt sur capteur de contact
	while(!terminate)
	{
		if(SensorValue[touchSensor] == 1)  // si appui sur capteur
		{
			terminate = true;
		}
		wait1Msec(50);
	}

	motor[motorA] = 0;
	playSound(soundDownwardTones);
	wait1Msec(1000);
}



// ============================================================================
// FONCTION menuPrincipal
// ============================================================================
void menuPrincipal()
{
	bool exitProgram = false;

	while(!exitProgram)
	{
		eraseDisplay();
		displayCenteredBigTextLine(1, "=== MENU ===");
		displayTextLine(3, "Droite : Mode Boussole");
		displayTextLine(4, "Gauche : Robot Fixe");
		displayTextLine(5, "Centre : Quitter");

		if(getButtonPress(buttonRight))
		{
			while(getButtonPress(buttonRight)) wait1Msec(10);
			boussoleMode();
		}
		else if(getButtonPress(buttonEnter))
		{
			while(getButtonPress(buttonEnter)) wait1Msec(10);
			exitProgram = true;
		}else if(getButtonPress(buttonLeft)) {
			while(getButtonPress(buttonLeft)) wait1Msec(10);
			FixeMode();
		}


		wait1Msec(100);
	}

	eraseDisplay();
	displayTextLine(3, "Programme termine");
	wait1Msec(1000);
}

// ============================================================================
// TÂCHE PRINCIPALE
// ============================================================================
task main()
{
	menuPrincipal();
}