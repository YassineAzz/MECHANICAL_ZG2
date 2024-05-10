//Programme arduino maquette zg2 avec capteur et moteur

// ----------- PINs ----------------------------------
  const uint8_t sensorPin = 5; //Out cpt position sur D5
  #define PWM1      6       // sortie D6 pin PWM
  #define EncodeurA 2

// ----------- Variables -----------------------------

  unsigned long freqLoop = 20;                //Hz, frequence de rafraichissement
  unsigned long periodeLoop = 1000/freqLoop; //ms, periode de rafraichissement

  unsigned long tps = 0;
  int rpm = 0;
  int16_t pos = 0;
  int completSimu = 0;
  String msg;
  float values[12]; // Tableau pour stocker les valeurs décimales

  //Variables envoyés par l'IHM
  int freq_acq = 0;
  int state_acq = 0;
  int state_mode_oscil = 0;
  int state_ctrl_mot = 0;
  float kp = 0.00; 
  float ki = 0.00;
  float kd = 0.00;
  int speed_order = 0;
  int min_speed_order = 0;
  int max_speed_order = 0;
  int nb_pas = 0; 
  int tps_simu = 0;

  float Q = 0.1; // Bruit du modèle (process noise)
  float R = 3.0; // Bruit de mesure (measurement noise)

  float x = 275; // État estimé
  float P = 0; // Erreur de covariance estimée

  //moteur
  int SpeedOrder;                 // variable entre potar entre 0 et 1023
  int RapportCycle;               // convertion en rapport de cycle
  int counter=0;                  // compter pour l'encodeur        
  int acc=15;                     //nb d'implusions avant calcul vitesse instantanee pour faire un tour

  unsigned long t0=0;                     // gestion du temps pour le prog
  unsigned long t1=0;                     // gestion du temps pour le prog
  unsigned long deltat=0;                 // temps pour la vitesse instantanee

  float RedMoteur= 50;        // rapport de reduction du moteur
  int ImpTour=16;             // nombre d'impulsion pour 1 tour 64/4
  float DeltaPos=0;           // nombre de tour depuis le dernier test
  float SpeedMoteur=0;        // vitesse actuel du moteur
  float Vref=0;               // Vitesse de la commande
  float Vrecu=0;
  float Vinst=0;              // vitesse instantanee
  float Vreel=0;              // vitesse reel moteur au moment du PI

  //Init valeur PI
  float E0=0;
  float I=0;
  float U=0;                      //commande
  float Umax=12;                  // Valeur max de la commande 
  float Umin =0;                  // Valeur min de la commande 
  int MinMoteur = 0;//50;             // Valeur minimum pour la vitesse de rotation du moteur (0 arret, 255 max)



void setup() {
  Serial.begin(115200); // Init Moniteur série
  //pre set capteur
  int i = 0;

  pinMode(PWM1, OUTPUT);
  pinMode(EncodeurA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EncodeurA), Encodeur, RISING);

  while( i <= 40){
    Capteur();
    delay(10);
    i += 1;
  }
}

// Mise en place du temps
  unsigned long currentMillis = millis();
  unsigned long refreshMillis = currentMillis;

void loop() {

  // Mise en place d'un rafraichissment
    while((currentMillis - refreshMillis) < periodeLoop)
    {
        currentMillis = millis();
    }
    refreshMillis = currentMillis;

  //Corps du programme, s'effectue à chaque période
    lectureIHM(); //lecture de la trame envoyée depuis l'ihm
    
    if(state_acq == 1){

      if(state_mode_oscil == 0){
         Capteur(); 
      }
      else if(state_mode_oscil == 1){
        Capteur();
        moteur();
      }
    
      envoiDonnees();
      tps += periodeLoop; 
      if (tps > (tps_simu*1000)){
        state_acq = 11;
        completSimu = 1; 
      }  
    }
    else if(state_acq == 11){
      Capteur();
      stopMoteur();
      envoiDonnees();
      state_acq = 2;
    }
    else if(state_acq == 2){
      tps = 0;
      rpm = 0;
      completSimu = 0;
      stopMoteur();
    }
  //Fin du corps du programme

}//Fin loop


// --------------- Fonctions ------------------------------------

  int toFloatArray(String str, float arr[], int size) {
    int count = 0; // Compteur pour le nombre de valeurs extraites
    char* token = strtok(const_cast<char*>(str.c_str()), ","); // Divise la chaîne de caractères en sous-chaînes

    while (token != NULL && count < size) {
      arr[count++] = atof(token); // Convertit chaque sous-chaîne en nombre décimal et l'ajoute au tableau
      token = strtok(NULL, ",");
    }

    return count; // Retourne le nombre de valeurs extraites
  }

  void envoiDonnees(void){
    Serial.print("T");
    Serial.print(tps);
    Serial.print("R");
    Serial.print(rpm);
    Serial.print("P");
    Serial.print(pos);
    Serial.print("C");
    Serial.println(completSimu);
  }

  void lectureIHM(void){
    if (Serial.available()) //Lecture du port série 
    {
      String message = Serial.readStringUntil('/'); // Lit la chaîne de caractères jusqu'au caractère '/'    

      int numValues = toFloatArray(message, values, 12);

      //Mise à jour des paramètres en fonction des valeurs reçues
      freq_acq = int(values[0]);
      state_acq = int(values[1]);
      state_mode_oscil = int(values[2]);
      state_ctrl_mot = int(values[3]);
      kp = values[4]; 
      ki = values[5];
      kd = values[6];
      speed_order = int(values[7]);
      min_speed_order = int(values[8]);
      max_speed_order = int(values[9]);
      nb_pas = int(values[10]); 
      tps_simu = int(values[11]);

      //Mise à jour de la fréquence de rafraichissement
      freqLoop = (unsigned long)freq_acq;
      periodeLoop =  1000/freqLoop;
    }
  }

  void Capteur(void){

    int16_t t = pulseIn(sensorPin, HIGH);
    if (t != 0 && t <= 1850)
    {
        int16_t d = (t - 1000) * 2; // Convertir la largeur d'impulsion en microsecondes en distance en millimètres.
        if (d < 0) { d = 0; } // Limiter la distance minimale à 0.
        //Filtrage de la valeur
        float x_pred = x;// Prédiction de l'état suivant
        float P_pred = P + Q;
        float K = P_pred / (P_pred + R);// Calcul du gain de Kalman
        x = x_pred + K * (d - x_pred);// Mise à jour de l'état estimé et de l'erreur de covariance estimée
        P = (1 - K) * P_pred;
        pos = x; //Position filtrée
    }
  }

  void moteur(){
    Vreel=Vinst;
    rpm = Vref*60;//Vreel;
    //Serial.print("Vreel= "); Serial.print(Vreel*60); Serial.println(" tr/min");
    Vref= map(speed_order, 0,330, 0,255); 
    //Serial.print("Vref= "); Serial.print(order); Serial.println(" tr/min");

    //Calcul de l'erreur
    E0= Vref-(Vreel*60);//Calcul du PI
    I=I+(periodeLoop*E0)/1000;//1000 conversion Te en s
    U=kp*E0+ki*I;
    if (U>Umax)
    {
      U=Umax;
    }
    else if (U<Umin)
    {
      U=Umin;
    }
    RapportCycle=map(U, Umin,Umax, 0,255);
    analogWrite(PWM1,RapportCycle); //affecter au moteur le rapport de cycle
  }

  void stopMoteur(){
    analogWrite(PWM1,0);
  }

  void Encodeur() {
  counter++; 
  if(counter==(2*acc)){
    t1=micros();
    deltat=t1-t0; 
    Vinst=(1e6*acc)/(deltat*ImpTour*RedMoteur*2); //vitesse instantanee de la sortie en tour/s
    t0=t1;
    counter=0;
  }
}