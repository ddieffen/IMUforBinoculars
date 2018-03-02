// Classe utilisée pour réaliser les calculs de quaternions et d'attitude
class CalculVectoriel
{
  private:
    double pitch, roll, azimut;
    double x, y;
    double ax, ay, az;
    double mx, my, mz;
    int dim = 20;  // number of elements for average 
  
  
  public:
  // Variables publiques
  
  // Methodes publiques
  void TraiterMesure(double ax, double ay, double az, double mx,double my, double mz);
  double Pitch();
  double Roll();
  double Azimut();
  void Trace();
};

void CalculVectoriel::TraiterMesure(double vax, double vay, double vaz, double vmx, double vmy, double vmz){
  // Les axes doivent être redressés dans un même repère hortonormé en amont du module

  //CalculVectoriel and filter
  // to stabilize the result

  double an, mn;
  an = sqrt(vax * vax + vay * vay + vaz * vaz);
  mn = sqrt(vmx * vmx + vmy * vmy + vmz * vmz);
  ax = ax * (dim -1)/dim + vax / dim / an;
  ay = ay * (dim -1)/dim + vay / dim / an;
  az = az * (dim -1)/dim + vaz / dim / an;
  mx = mx * (dim -1)/dim + vmx / dim / mn;
  my = my * (dim -1)/dim + vmy / dim / mn;
  mz = mz * (dim -1)/dim + vmz / dim / mn;

  pitch = asin(ax / sqrt(ax * ax + ay * ay + az * az));
  
  // Définition du roll dans le plan (y,z)
  if ((ay * ay + az * az) <= 0.000000001) {
    roll = 0;
  }else{
    roll = asin(ay / sqrt( ay *ay + az * az ));
  }
  
  y = my * cos(roll) - mz * sin(roll);
  x = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch);
  azimut = atan2(y,x);
}

void CalculVectoriel::Trace(){
  Serial.print("Mesure ax, ay, az ; ");
  Serial.print(ax, 6);
  Serial.print(" ; ");
  Serial.print(ay, 6);
  Serial.print(" ; ");
  Serial.print(az, 6);
  Serial.print(" ; Mesure mx, my, mz ; ");
  Serial.print(mx, 6);
  Serial.print(" ; ");
  Serial.print(my, 6);
  Serial.print(" ; ");
  Serial.print(mz, 6);
  Serial.print(" ; x, y  ; ");
  Serial.print(x);
  Serial.print(" ; ");
  Serial.print(y);
  Serial.print(" ; Azimut  ; ");
  Serial.print(Azimut());
  Serial.print(" ; Pitch ; ");
  Serial.print(Pitch());  
  Serial.print(" ; Roll ; ");
  Serial.println(Roll());  
}

double CalculVectoriel::Pitch(){
  return - pitch * 180.0f / PI;
}
double CalculVectoriel::Roll(){
  return roll * 180.0f / PI;
}
double CalculVectoriel::Azimut(){
  if (azimut > 180){
    return azimut * 180.0f / PI -180;  //0 = Nord, 360 = Nord. Le magnétomètre pointe vers le sud.
  }else {
    return azimut * 180.0f / PI + 180;  //0 = Nord, 360 = Nord. Le magnétomètre pointe vers le sud.
  }
}
