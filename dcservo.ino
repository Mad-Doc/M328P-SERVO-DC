/*
  Ce programme utilise un Arduino nano pour controler un moteur DC a balais avec un encodeur a quadrature pour un controle de position
  précis, et "commander" en STEP et DIR par deux entrée sur le nano et une sortie HOME "sensorless".
  le port serie est utilisé pour configurer le controlleur, au démmarage l'aide s'affiche une fois, 
  si pas appuyer sur "H" puis faite Enter.
  ce code fonctionne bien avec des signaux step rapide (attention limite a vérifier, si trop de vitesse = nano se coupe!).
  code originalement crée par Misan (Miguel Sanchez),et modifier par Mad-doc.
  Ajout de plusieurs fonctions, de sécurité, d'une fonction homing sans capteur, d'une deadband réglable,
  d'un antiwindup pour stabilisé le pid ,ect .
  dans un futur, une fonction autotune sera implantée, entre temps il est recommander d'utilisé le software de calibration 
  spécifiquement dévelloper pour ce code!
  pour toute information complementaire, veuiller lire le Readme.txt fournis!   
  
  pinout utiliser
  entrée D3 & D4 sont connectée a l'encodeur directement ou via un ls7141 en option, signal A et B.
  entrée D2 est l'entrée STEP provenant de votre controlleur (Marlin, Klipper, GRBL, ect)
  entrée D6 est l'entrée DIR provenant de votre controlleur (Marlin, Klipper, GRBL, ect)
  entrée D5 est l'entrée enable provenant de votre controlleur (Marlin, Klipper, GRBL, ect)
  sortie D7 est la sortie HOME pour le homing sans capteur "HomeFunc" 
  sortie D9 et D10 sont des sortie PWM pour le controle du pont en H de votre choix !
  attention D10 controle le sens positif et D9 le sens négatif ! 
  si le moteur se met a tourner non stop au premier branchement, il faut simplement inversé D9 et D10 
  sur les entrée de votre pont en H ou les signaux d'entrée A et B de l'encodeur , au choix.
  Attention, les gains PID sont ici par défaut, il faudra les ajuster selon votre moteur. 
  le courant maximum au moteur est réglable de 0 a 100% au besoin. 
  la fonction homing "HomeFunc" permet de faire un homing sans capteur, la sensibilité se regle aussi de 0 a 100%,
  il est recommander que la butée pour le homing sensorless soit "souple" ,donc l'axe doit toucher une butée en caoutchouk,
  et que cette buttée soit suffisament solide.
  !selon le moteur et sa charge réel en fonctionnement, le réglage doit etre le plus bas possible pour une efficacité sure!
  si le resultat n'est pas exploitable, ou si l'axe utilise une forte démultiplication (vis trapésoidale ou a bille,
  Gros réducteur, ect) l'utilisation d'un capteur réel de homing est recommandé!.
  Version Pré finale du drivers !

*/
#include <EEPROM.h>
#include <PID_v2.h>
#define encoder0PinA  3  // entrée encodeur A (ou pulse input si ls7141)
#define encoder0PinB  4  // entrée encodeur B (ou sens input si ls7141)
#define M1            10 // sortie pwm moteur vers Hbridge HIGH 
#define M2            9  // sortie pwm moteur vers Hbridge LOW
#define enable        5  // entrée enable 
#define ENDSTOP       7  // sortie homing sans capteur
#define STATUS        8  // satus du drivers servo
#define STEP          2  // entrée Step
#define DIR           6  // entrée Dir
#define CT            A6 // entrée mesure du courant moteur

/*--------------software tuning--------------*/
byte pos[1000]; //reserve 1000 octet pour la mesure de réaction du moteur
int x=0; 
byte skip=5;

/*----------------section pid----------------*/
double kp=5,ki=0,kd=0.00; //valeur initialisation PID avant chargement eeprom
double kpd=01.0000, kid=000.0000, kdd=000.0000; //valeur par défaut pid en cas d'instabilité moteur
double input=0; 
double output=0;
double setpoint=0;
PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);

/*--------------section automatique----------*/
bool auto1 = false;
bool auto2 = false;
bool counting = false;
int iterationCount = 0; 

/*--------------section encodeur-------------*/
volatile long encoder0Pos = 0;
long target1 = 0;  

/*-----------------HomeFunc------------------*/
bool isHome = true;
float power = 100.00; //définit la puissance maximum au moteur  0 a 100 exprimé en % ! 
float Sens = 000.30; //Sensibilité de détection ,0% a 100%
int curVal = 0; 
float percentAmp = 0;
bool homeState = LOW; 
/*--------------entrée enable----------------*/
bool enabled = false;

/*---------activation pin interruption-------*/
void pciSetup(byte pin){
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // entrée enable
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group 
}

/*---------code principale------------------*/
void setup() { 
  pinMode(encoder0PinA, INPUT);          //entrée encodeur A ou PULSE 
  pinMode(encoder0PinB, INPUT);          //entrée encodeur B ou DIR 
  pinMode(enable, INPUT);                //entrée enable, ON = +5V in 
  pinMode(ENDSTOP, OUTPUT);              //sortie homing
  pinMode(STATUS, OUTPUT);               //led d'état uniquement
  pinMode(STEP, INPUT);                  //entrée STEP 
  pinMode(DIR, INPUT);                   //entrée DIR
  pinMode(CT, INPUT);                    //mesure du courant
  pciSetup(encoder0PinB);                //sens du comptage
  attachInterrupt(1, encoderInt, CHANGE);// encoder pin on interrupt 1 - pin 3 ,comptage encodeur
  attachInterrupt(0, countStep, RISING); // step  input on interrupt 0 - pin 2 ,comptage STEP
  TCCR1B = TCCR1B & 0b11111000 | 1;      // set 31Khz PWM motor frequency 
  Serial.begin (115200);                 // active le port serie
  help();                                // affiche l'aide dans la console serie
  recoverPIDfromEEPROM();                // charge le pid défini en eeprom

/*-----------configure le pid----------------*/ 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255,255);
/*----initialise la sortie homing a zero-----*/
  digitalWrite(ENDSTOP,LOW);             
} 
void loop(){
    calculPID();
    HomeFunc();
    if(Serial.available()) process_line(); //n'active le port serie que si connecter pour gagne du temps cpu !
    if(auto1) if(millis() % 1000 == 0) target1=random(-3000, 3000); // test position aléatoire automatique,DANGER!
    if(auto2) if(millis() % 1000 == 0) printPos();           //affiche la position encodeur toute les seconde!
    if(counting &&  (skip++ % 1)==0 ) {pos[x]=encoder0Pos; if(x<999) x++; else counting=false;} //software tuning
}
void HomeFunc(){          //fonction homing sensorless ! 
 curVal = analogRead(CT); //mesure de courant , valeur numérique entre 0 et 1023 ! 
 percentAmp = (curVal * 0.097751711); //convertit la mesure digital en pourcentage
  if (percentAmp >= Sens) {
    homeState = HIGH; 
  } else if (percentAmp < Sens) { 
    homeState = LOW; 
  }
  digitalWrite(ENDSTOP, homeState);
}
void calculPID(){
    if (digitalRead(enable)){
      enabled = true;
      input = encoder0Pos; 
      setpoint=target1;
      myPID.SetMode(AUTOMATIC);
      if(input==setpoint)pwmOut(0); else pwmOut(output*(power/100)); //deadband a inserer ici ?
      while(!myPID.Compute()){digitalWrite(STATUS, HIGH);}
    }else {
      enabled = false;
      encoder0Pos=0;
      setpoint=0;
      target1=0;
      digitalWrite(STATUS, LOW);
    }
}
void pwmOut(int out) {
   if(out<0) { digitalWrite(M1,0); analogWrite(M2,abs(out)); }
   else { digitalWrite(M2,0); analogWrite(M1,abs(out)); }
}
const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; //matrice d'encodage en quadrature
static unsigned char New, Old;
ISR (PCINT2_vect) { 
  Old = New << 2;
  New = (digitalRead(encoder0PinA) << 1) | digitalRead(encoder0PinB);
  encoder0Pos += QEM[Old | New];
}
void encoderInt() { 
  Old = New << 2;
  New = (digitalRead(encoder0PinA) << 1) | digitalRead(encoder0PinB);
  encoder0Pos += QEM[Old | New];
}
void countStep(){ 
  if (digitalRead(DIR)) 
  target1--;
  else 
  target1++;
  } 
void process_line() {
  static String inputString = ""; 
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') { 
      inputString.trim(); 
      if (inputString.length() > 0) {
        char cmd = inputString[0]; 
        if (cmd > 'Z') cmd -= 32;  
        String value = inputString.substring(1); 
        switch (cmd) {
          case 'P': kp = value.toFloat(); update_pid(); Serial.print("p: "); Serial.println(kp, 4); break;
          case 'I': ki = value.toFloat(); update_pid(); Serial.print("i: "); Serial.println(ki, 4); break;
          case 'D': kd = value.toFloat(); update_pid(); Serial.print("d: "); Serial.println(kd, 4); break;
          case 'X': target1 = value.toInt(); x = 0; counting = true; for (int i = 0; i < 300; i++) pos[i] = 0; break;
          case 'C': target1 = value.toInt(); Serial.print("Allez a: "); Serial.print(target1); Serial.println(" Pulse."); break;
          case 'T': auto1 = !auto1; break;
          case 'L': auto2 = !auto2; break;
          case 'A': basepid(); update_pid(); break;
          case 'Q': printPos(); break;
          case 'V': Valueactive(); break;
          case 'H': help(); break;
          case 'W': writetoEEPROM(); break;
          case 'K': eedump(); break;
          case 'R': recoverPIDfromEEPROM(); update_pid(); break;
          case 'S': for (int i = 0; i < x; i++) Serial.println(pos[i]); break;
          case 'Z': Sens = value.toFloat(); Serial.print(" Sensibilité Homing: "); Serial.print(Sens); Serial.println(" %"); break;
          case '&': opt1(); break; //demande du code de vérification "dongle" (trouver une chaine spécifique a recevoir ! )
          case 'O': power = value.toFloat(); Serial.print("  Puissance moteur: "); Serial.print(power); Serial.println(" %"); break;
          default: Serial.println(" ! Commande inconnue !"); break;
        }
      }
      inputString = ""; 
    } else {
      inputString += c; 
    }
  }
}
void Valueactive(){ 
  
  Serial.println(F("                 Valeur active du PID                 "));
    Serial.print(F(" P = "));
  Serial.println(kp, 4);
    Serial.print(F(" I = "));
  Serial.println(ki, 4);
    Serial.print(F(" D = "));
  Serial.println(kd, 4); 
  Serial.println(F("                    Valeur HomeFunc                   "));
    Serial.print(F(" Puissance moteur   : "));
  Serial.print(power, 2);
  Serial.println(F(" %                        "));
    Serial.print(F(" Sensibiliter Homing: "));
  Serial.print(Sens, 2);
  Serial.println(F(" %                          ")); 
  Serial.println(F("                    Etat du driver                    "));
  Serial.print(F(" Entrée Enabled: "));
  if (enabled == true){ Serial.println("ON"); } else{ Serial.println("OFF"); }
}
void opt1(){ //code de vérification pour activation du programme software tuning + n° de modele !
  Serial.println(123456789); //valeur a changer pour avoir un code hexadécimal en caractere acsii ! 
  Serial.println("M328pDCservo"); //indique le model de controlleur !
}
void update_pid(){
  myPID.SetTunings(kp,ki,kd);
}
void printPos() {
  Serial.print(iterationCount);
  Serial.print(F(" : Position = ")); Serial.print(encoder0Pos); 
  Serial.print(F(" PID_output = ")); Serial.print(output); 
  Serial.print(F(" Target = ")); Serial.println(setpoint);
  iterationCount++;
}
void help() {
  Serial.println(F("     Controleur moteur DC PID entrée STEP et DIR      "));
  Serial.println(F("        crée par Misan, remodeler par Maddoc          "));
  Serial.println(F("                                                      "));
  Serial.println(F("        < Liste des commande disponible >             "));
  Serial.println(F("                                                      "));
  Serial.println(F(" P = gain proportionel, exemple P123.34               "));
  Serial.println(F(" I = gain integral, exemple I123.34                   "));
  Serial.println(F(" D = gain derivé, exemple D123.34                     "));
  Serial.println(F(" Q = position encodeur et état sortie PWM             "));
  Serial.println(F(" C = destination moteur, exemple C123 = position 123  "));
  Serial.println(F(" T = séquence mouvement aléatoire du moteur  !DANGER! "));
  Serial.println(F(" V = affiche les valeur active                        ")); 
  Serial.println(F(" W = enregistre les valeurs dans l'eeprom             ")); 
  Serial.println(F(" R = charge les valeurs de l'eeprom en ram            "));
  Serial.println(F(" A = charge un PID SAFE en cas d'instabiliter !       "));
  Serial.println(F(" L = active la position en continu, reappuyer = stop  ")); 
  Serial.println(F(" Z = Sensibilité du Homing ,Valeur de 0.00% a 100.00% "));
  Serial.println(F(" O = Force max du moteur 0%=null, 50%=médian 100%=max "));
  Serial.println(F(" H = affiche cette aide.                              ")); 
  Serial.println(F(""));
}
void basepid() { //valeur pid defaut en cas de moteur trop instable !
  kp =kpd;
  ki =kid;
  kd =kdd;
  Serial.println(F("             PID par défaut charger                   "));
  Serial.println(F(""));
}
void writetoEEPROM() {
  double oldKp, oldKi, oldKd, oldSens, oldPower;
  EEPROM.get(0, oldKp);
  EEPROM.get(4, oldKi);
  EEPROM.get(8, oldKd);
  EEPROM.get(12, oldSens);
  EEPROM.get(16, oldPower);

  if (oldKp != kp || oldKi != ki || oldKd != kd || oldSens != Sens || oldPower != power) { 
    EEPROM.put(0, kp);
    EEPROM.put(4, ki);
    EEPROM.put(8, kd);
    EEPROM.put(12, Sens);
    EEPROM.put(16, power);
    double cks = kp + ki + kd + Sens + power;
    EEPROM.put(20, cks);
    Serial.println(F("             Valeur enregistrée en EEPROM             "));
    Serial.println(F(""));
  } else {
    Serial.println(F("         Valeur inchangée, pas d'écriture EEPROM      "));
    Serial.println(F(""));
  }
}
void recoverPIDfromEEPROM() {
  double cks, cksEE;
  EEPROM.get(0, kp);
  EEPROM.get(4, ki);
  EEPROM.get(8, kd);
  EEPROM.get(12, Sens);
  EEPROM.get(16, power);
  EEPROM.get(20, cksEE);
  cks = kp + ki + kd + Sens + power; 
  if (cks == cksEE) {
    Serial.println(F("           Valeur EEPROM trouvée et chargée           "));
    Serial.println(F(""));
    myPID.SetTunings(kp, ki, kd);
  } else {
    Serial.println(F("      !!! Valeur EEPROM erronée ou manquante !!!      "));
    Serial.println(F(""));
  }
}
void eedump() { //lit les valeur hexa de l'eeprom (pour soft control uniquement!)
 for(int i=0; i<32; i++) { Serial.print(EEPROM.read(i),HEX); Serial.print(" "); }Serial.println(); 
}
