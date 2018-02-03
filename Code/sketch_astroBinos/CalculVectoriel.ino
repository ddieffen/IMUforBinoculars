/* Classe Calcul Vectoriel
*/
class CalculVectoriel
{
  private:
    // Variables privées
    float phi = 0.0 ; // angle de rotation
    float phii, phij ; // angle de rotation décomposé sur i, j, k (normalement k = 0)
    float ri, rj, rk;      // vecteur de rotation permettant de définir un référentiel horizontal
    //  float a[3];      //vecteur acceleration (x, y, z)
    float mi, mj, mk;      //vecteur magnitude sur le plan horizontal (i, j, k)
    float ax, ay, az;
    float mx, my, mz;
    float thetax, thetam, theta;
    ;

  public:
    // Variables publiques

    // Methodes publiques
    //void normalise(float ax, float ay, float az);
    void TraiterMesure(float ax, float ay, float az, float mx, float my, float mz);
    void DefinirRotation(float ax, float ay, float az);
    void DefinirMagnitudeHorizontale(float mx, float my, float mz);
    float Azimut();
    float Altitude();
    void Trace();
};
void CalculVectoriel::Trace() {
  Serial.print("Mesure ax ; ");
  Serial.print(ax, 6);
  Serial.print(" ; Mesure ay ; ");
  Serial.print(ay, 6);
  Serial.print(" ; Mesure az ; ");
  Serial.print(az, 6);
  Serial.print(" ; Mesure mx ; ");
  Serial.print(mx, 6);
  Serial.print(" ; Mesure my ; ");
  Serial.print(my, 6);
  Serial.print(" ; Mesure mz ; ");
  Serial.print(mz, 6);
  Serial.print(" ; Azimumt  ; ");
  Serial.print(Azimut());
  Serial.print(" ; Altitude ; ");
  Serial.println(Altitude());
  Serial.print (" phii, phij ; ");
  Serial.print (phii, 6);
  Serial.print (" ; ");
  Serial.print (phij, 6);
  Serial.print (" ; ");
  Serial.print (" ri, rj, rk, phi ; ");
  Serial.print (ri, 6);
  Serial.print (" ; ");
  Serial.print (rj, 6);
  Serial.print (" ; ");
  Serial.print (rk, 6);
  Serial.print (" ; ");
  Serial.print (phi, 6);
  Serial.print ("; mi, mj, mk : ");
  Serial.print (mi, 6);
  Serial.print (" ; ");
  Serial.print (mj, 6);
  Serial.print (" ; ");
  Serial.print (mk, 6);
  Serial.print ("; phix, phim ; ");
  Serial.print (thetax, 6);
  Serial.print (" ; ");
  Serial.print (thetam, 6);

}


void CalculVectoriel::TraiterMesure(float ax, float ay, float az, float mx, float my, float mz) {
  DefinirRotation(ax, ay, az);
  DefinirMagnitudeHorizontale(mx, my, mz);
}

float CalculVectoriel::Altitude() {
  // L'altitude des jumelles, exprimée sur 360° correspond à l'angle entre le vecteur acceleration a et l'axe Z sur le plan (x, z)
  // O° = horizon
  // 90° = verticale
  return -phij * 180.0f / PI;
}
float CalculVectoriel::Azimut() {
  float xi, xj ; //, xk; // vecteur x(1,0,0) exprime sur le referentiel géocentré i, j, k
  //  float thetax, thetam, theta;
  // L'azimut correspond à l'angle (360°) entre l'axe x projeté sur le plan (i,j) et le vecteur gravité projeté sur le plan (i,j)

  // Coordonnées de x sur le repere i, j, k (géocentré)
  xi = cos(phi) + (1.0f - cos(phi)) * (ri * ri );
  xj = (1.0f - cos(phi)) * (ri * rj) + sin(phi) * ( rk);
  //xk = (1-cos(phi))*(ri*rk) + sin(phi)*(-rj);

  // Transformation cooronnées cartésiennes vers coordonnées sphériques
  // Formules sur https://fr.wikipedia.org/wiki/Coordonnées_sphériques
  if (xj >= 0) {
    thetax = acos(xi / sqrt(xi * xi + xj * xj) );
  } else {
    thetax = 2.0f * PI - acos(xi / sqrt(xi * xi + xj * xj));
  }
  if (mj >= 0) {
    thetam = acos(mi / sqrt(mi * mi + mj * mj) );
  } else {
    thetam = 2.0f * PI - acos(mi / sqrt(mi * mi + mj * mj) );
  }

  theta = thetam - thetax;
  if (theta > PI) {
    return ((theta - 2.0f * PI)  * 180.0f / PI);
  } else if (theta < -PI) {
    return (theta + 2.0f * PI) * 180.0f / PI;
  } else {
    return theta * 180.0f / PI;
  }
}

void CalculVectoriel::DefinirRotation(float vax, float vay, float vaz) {
  // On défini la rotation (phi, r[]) qui permet d'obtenir le plan horizontal
  // les vecteurs i et j sont les deux vecteurs du plan horizontal
  // k est le vecteur vertical
  // Le vecteur a définit la verticale z.
  // on cherche à définir la rotation permettant de passer du repere x, y, z à i, j, k
  // Voir les formules sur https://fr.wikipedia.org/wiki/Rotation_vectorielle, chapitre "Composition de deux rotations vectorielles"

  //  float norm ; //Valeur de normalisation du vecteur a
  //  norm = sqrt(ax * ax + ay * ay + az * az);
  ax = vax;
  ay = vay;
  az = vaz;

  // Projection de ay sur l'axe z => rotation sur l'axe i
  phii  = -asin(ay / sqrt(ay * ay + az * az)) ;
  // Projection de ax sur l'axe z => rotation sur l'axe j
  phij = -asin(ax / sqrt(ax * ax + az * az)) ;

  phi = 2.0f * acos(cos(phii / 2.0f) * cos(phij / 2.0f));
  ri = cos(phij / 2.0f) * sin(phii / 2.0f) / sin(phi / 2.0f);
  rj = cos(phii / 2.0f) * sin(phij / 2.0f) / sin(phi / 2.0f);
  rk = sin(phii / 2.0f) * sin(phij / 2.0f) / sin(phi / 2.0f);
}

void CalculVectoriel::DefinirMagnitudeHorizontale(float vmx, float vmy, float vmz) {

  /*
     // Formules sur  https://fr.wikipedia.org/wiki/Rotation_vectorielle
     x, y, z = vecteur champ magnétique a tranformer en mi, mj, mk
     phi, nx, ny, nz le vecteur rotation défini précédemment
     mi = COS(phy)*x + (1-COS(phy))*(nx*nx*x+nx*ny*y+nx*nz*z)+SIN(phy)*(-nz*y+ny*z)
     mj = COS(phy)*y + (1-COS(phy))*(nx*ny*x + ny*ny*y + ny*nz*z) + SIN(phy)*(nz*x -nx*z)
     mk = COS(phy)*z + (1-COS(phy))*(nx*nz*x+ny*nz*y+nz*nz*z)+SIN(phy)*(-ny*x + nx*y)

  */
  mx = vmx;
  my = vmy;
  mz = vmz;

  this->mi = cos(phi) * mx + (1.0f - cos(phi)) * (ri * ri * mx + rj * ri * my + rk * ri * mz) + sin(phi) * ( 0     - rk * my + rj * mz);
  this->mj = cos(phi) * my + (1.0f - cos(phi)) * (ri * rj * mx + rj * rj * my + rk * rj * mz) + sin(phi) * ( rk * mx + 0     - ri * mz);
  this->mk = cos(phi) * mz + (1.0f - cos(phi)) * (ri * rk * mx + rj * rk * my + rk * rk * mz) + sin(phi) * (-rj * mx + ri * my + 0 * mz);
}
;

